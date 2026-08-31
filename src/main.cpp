#include "DomainParticipantFactory.h"
#include "DomainParticipant.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "Topic.h"
#include "mission.h"
#include "missionDataReader.h"
#include "missionDataWriter.h"
#include "missionTypeSupport.h"
#include "task_plan.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr DDS::DomainId_t kDomainId = 42;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kReferenceLatDeg = 31.2304;
constexpr double kReferenceLonDeg = 121.4737;

long long now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void settle(unsigned int milliseconds = 180) {
    ZRSleep(milliseconds);
}

void check(DDS::ReturnCode_t code, const char* operation) {
    if (code != DDS::RETCODE_OK) {
        std::ostringstream stream;
        stream << operation << " failed with DDS return code " << code;
        throw std::runtime_error(stream.str());
    }
}

void set_text(DDS_Char*& destination, const std::string& value) {
    DDS_StringFinalize(&destination);
    const DDS_Char* input = value.c_str();
    if (!DDS_StringInnerCopyEx(&destination, &input, NULL)) {
        throw std::runtime_error("Unable to allocate DDS string");
    }
}

std::string text_or_empty(const DDS_Char* value) {
    return value == NULL ? "" : value;
}

struct Pose {
    double east_m = 0.0;
    double north_m = 0.0;
    double up_m = 0.0;
    double heading_deg = 0.0;
};

Pose as_pose(const LocalPose& value) {
    return {value.east_m, value.north_m, value.up_m, value.heading_deg};
}

void set_pose(LocalPose& destination, const Pose& source) {
    destination.east_m = source.east_m;
    destination.north_m = source.north_m;
    destination.up_m = source.up_m;
    destination.heading_deg = source.heading_deg;
}

double distance_m(const Pose& a, const Pose& b) {
    const double east = a.east_m - b.east_m;
    const double north = a.north_m - b.north_m;
    const double up = a.up_m - b.up_m;
    return std::sqrt(east * east + north * north + up * up);
}

Pose lerp(const Pose& start, const Pose& end, double fraction) {
    return {
        start.east_m + (end.east_m - start.east_m) * fraction,
        start.north_m + (end.north_m - start.north_m) * fraction,
        start.up_m + (end.up_m - start.up_m) * fraction,
        start.heading_deg + (end.heading_deg - start.heading_deg) * fraction,
    };
}

struct TaskInfo {
    std::string id;
    std::string incident_id;
    std::string capability;
    std::string predecessor;
    TaskKind kind = TASK_SURVEY;
    TaskPriority priority = PRIORITY_NORMAL;
    double payload_kg = 0.0;
    long deadline_s = 0;
    Pose target;
    GeoPoint target_wgs84{};
};

TaskInfo as_task_info(const TaskRequest& value) {
    TaskInfo task;
    task.id = text_or_empty(value.task_id);
    task.incident_id = text_or_empty(value.incident_id);
    task.capability = text_or_empty(value.required_capability);
    task.predecessor = text_or_empty(value.predecessor_task_id);
    task.kind = value.kind;
    task.priority = value.priority;
    task.payload_kg = value.payload_kg;
    task.deadline_s = value.deadline_s;
    task.target = as_pose(value.target_enu);
    task.target_wgs84 = value.target_wgs84;
    return task;
}

struct BidInfo {
    std::string id;
    std::string task_id;
    std::string agent_id;
    bool feasible = false;
    double eta_s = 0.0;
    double cost = 0.0;
    long bid_ttl_ms = 0;
    long long created_at_ms = 0;
    std::string rationale;
};

BidInfo as_bid_info(const CandidateBid& value) {
    return {
        text_or_empty(value.bid_id),
        text_or_empty(value.task_id),
        text_or_empty(value.agent_id),
        value.feasible != false,
        value.eta_s,
        value.cost_score,
        value.bid_ttl_ms,
        value.created_at_ms,
        text_or_empty(value.rationale_code),
    };
}

struct CapabilityInfo {
    std::string agent_id;
    std::string capability;
    bool available = false;
    Pose home;
    double speed_mps = 0.0;
    double battery_percent = 0.0;
};

CapabilityInfo as_capability_info(const VehicleCapability& value) {
    return {
        text_or_empty(value.agent_id),
        text_or_empty(value.primary_capability),
        value.available != false,
        as_pose(value.home_enu),
        value.cruise_speed_mps,
        value.battery_percent,
    };
}

struct Profile {
    bool reliable;
    bool transient_local;
    int history_depth;
};

const Profile kMissionCommand{true, true, 32};
const Profile kCapability{true, true, 32};
const Profile kBidWindow{true, false, 32};
const Profile kTelemetry{false, false, 10};
const Profile kAuditEvent{true, true, 128};

class DdsNode {
public:
    explicit DdsNode(const std::string& name) : name_(name) {
        DDS::DomainParticipantFactory* factory = DDS::DomainParticipantFactory::get_instance();
        if (factory == NULL) {
            throw std::runtime_error("Unable to get ZRDDS DomainParticipantFactory");
        }

        participant_ = factory->create_participant(
            kDomainId, DDS::DOMAINPARTICIPANT_QOS_DEFAULT, NULL, DDS::STATUS_MASK_NONE);
        if (participant_ == NULL) {
            throw std::runtime_error("Unable to create DDS participant for " + name_);
        }

        register_types();
        create_topics();

        publisher_ = participant_->create_publisher(
            DDS::PUBLISHER_QOS_DEFAULT, NULL, DDS::STATUS_MASK_NONE);
        subscriber_ = participant_->create_subscriber(
            DDS::SUBSCRIBER_QOS_DEFAULT, NULL, DDS::STATUS_MASK_NONE);
        if (publisher_ == NULL || subscriber_ == NULL) {
            throw std::runtime_error("Unable to create DDS pub/sub endpoints for " + name_);
        }
    }

    DdsNode(const DdsNode&) = delete;
    DdsNode& operator=(const DdsNode&) = delete;

    ~DdsNode() {
        shutdown();
    }

    DDS::Topic* task_topic() const { return task_topic_; }
    DDS::Topic* capability_topic() const { return capability_topic_; }
    DDS::Topic* bid_topic() const { return bid_topic_; }
    DDS::Topic* assignment_topic() const { return assignment_topic_; }
    DDS::Topic* state_topic() const { return state_topic_; }
    DDS::Topic* event_topic() const { return event_topic_; }
    DDS::Topic* transform_topic() const { return transform_topic_; }

    DDS::DataWriter* writer(DDS::Topic* topic, const Profile& profile) {
        DDS::DataWriterQos qos;
        check(publisher_->get_default_datawriter_qos(qos), "get writer qos");
        qos.reliability.kind = profile.reliable
            ? DDS_RELIABLE_RELIABILITY_QOS : DDS_BEST_EFFORT_RELIABILITY_QOS;
        qos.durability.kind = profile.transient_local
            ? DDS_TRANSIENT_LOCAL_DURABILITY_QOS : DDS_VOLATILE_DURABILITY_QOS;
        qos.history.kind = DDS_KEEP_LAST_HISTORY_QOS;
        qos.history.depth = profile.history_depth;
        DDS::DataWriter* result = publisher_->create_datawriter(
            topic, qos, NULL, DDS::STATUS_MASK_NONE);
        if (result == NULL) {
            throw std::runtime_error("Unable to create DDS writer for " + name_);
        }
        return result;
    }

    DDS::DataReader* reader(DDS::Topic* topic, const Profile& profile) {
        DDS::DataReaderQos qos;
        check(subscriber_->get_default_datareader_qos(qos), "get reader qos");
        qos.reliability.kind = profile.reliable
            ? DDS_RELIABLE_RELIABILITY_QOS : DDS_BEST_EFFORT_RELIABILITY_QOS;
        qos.durability.kind = profile.transient_local
            ? DDS_TRANSIENT_LOCAL_DURABILITY_QOS : DDS_VOLATILE_DURABILITY_QOS;
        qos.history.kind = DDS_KEEP_LAST_HISTORY_QOS;
        qos.history.depth = profile.history_depth;
        DDS::DataReader* result = subscriber_->create_datareader(
            topic, qos, NULL, DDS::STATUS_MASK_NONE);
        if (result == NULL) {
            throw std::runtime_error("Unable to create DDS reader for " + name_);
        }
        return result;
    }

    void shutdown() {
        if (participant_ == NULL) {
            return;
        }
        participant_->delete_contained_entities();
        DDS::DomainParticipantFactory::get_instance()->delete_participant(participant_);
        participant_ = NULL;
    }

private:
    void register_types() {
        check(TaskRequestTypeSupport::get_instance()->register_type(participant_, NULL), "register TaskRequest");
        check(VehicleCapabilityTypeSupport::get_instance()->register_type(participant_, NULL), "register VehicleCapability");
        check(CandidateBidTypeSupport::get_instance()->register_type(participant_, NULL), "register CandidateBid");
        check(TaskAssignmentTypeSupport::get_instance()->register_type(participant_, NULL), "register TaskAssignment");
        check(VehicleStateTypeSupport::get_instance()->register_type(participant_, NULL), "register VehicleState");
        check(ExecutionEventTypeSupport::get_instance()->register_type(participant_, NULL), "register ExecutionEvent");
        check(CoordinateTransformTypeSupport::get_instance()->register_type(participant_, NULL), "register CoordinateTransform");
    }

    DDS::Topic* topic(const char* topic_name, const char* type_name) {
        DDS::Topic* result = participant_->create_topic(
            topic_name, type_name, DDS::TOPIC_QOS_DEFAULT, NULL, DDS::STATUS_MASK_NONE);
        if (result == NULL) {
            throw std::runtime_error(std::string("Unable to create DDS topic ") + topic_name);
        }
        return result;
    }

    void create_topics() {
        task_topic_ = topic("Mission.TaskRequest", TaskRequestTypeSupport::get_instance()->get_type_name());
        capability_topic_ = topic("Mission.VehicleCapability", VehicleCapabilityTypeSupport::get_instance()->get_type_name());
        bid_topic_ = topic("Mission.CandidateBid", CandidateBidTypeSupport::get_instance()->get_type_name());
        assignment_topic_ = topic("Mission.TaskAssignment", TaskAssignmentTypeSupport::get_instance()->get_type_name());
        state_topic_ = topic("Mission.VehicleState", VehicleStateTypeSupport::get_instance()->get_type_name());
        event_topic_ = topic("Mission.ExecutionEvent", ExecutionEventTypeSupport::get_instance()->get_type_name());
        transform_topic_ = topic("Mission.CoordinateTransform", CoordinateTransformTypeSupport::get_instance()->get_type_name());
    }

    std::string name_;
    DDS::DomainParticipant* participant_ = NULL;
    DDS::Publisher* publisher_ = NULL;
    DDS::Subscriber* subscriber_ = NULL;
    DDS::Topic* task_topic_ = NULL;
    DDS::Topic* capability_topic_ = NULL;
    DDS::Topic* bid_topic_ = NULL;
    DDS::Topic* assignment_topic_ = NULL;
    DDS::Topic* state_topic_ = NULL;
    DDS::Topic* event_topic_ = NULL;
    DDS::Topic* transform_topic_ = NULL;
};

template <typename Reader, typename Sequence, typename Callback>
void drain(Reader* reader, Callback callback) {
    Sequence samples;
    DDS::SampleInfoSeq infos;
    const DDS::ReturnCode_t result = reader->take(
        samples, infos, LENGTH_UNLIMITED, DDS::ANY_SAMPLE_STATE,
        DDS::ANY_VIEW_STATE, DDS::ANY_INSTANCE_STATE);
    if (result == DDS::RETCODE_NO_DATA) {
        return;
    }
    check(result, "take DDS samples");
    for (DDS::ULong i = 0; i < infos.length(); ++i) {
        if (infos[i].valid_data) {
            callback(samples[i]);
        }
    }
    check(reader->return_loan(samples, infos), "return DDS loan");
}

template <typename Writer, typename Sample>
void publish(Writer* writer, const Sample& sample, const char* topic_name) {
    check(writer->write(sample, DDS::HANDLE_NIL_NATIVE), topic_name);
}

void publish_event(
    ExecutionEventDataWriter* writer,
    const std::string& event_id,
    const std::string& incident_id,
    const std::string& task_id,
    const std::string& source,
    EventKind kind,
    TaskPriority severity,
    const std::string& message_code) {
    ExecutionEvent value;
    ExecutionEventInitialize(&value);
    set_text(value.event_id, event_id);
    set_text(value.incident_id, incident_id);
    set_text(value.task_id, task_id);
    set_text(value.source_agent_id, source);
    value.kind = kind;
    value.severity = severity;
    set_text(value.message_code, message_code);
    value.occurred_at_ms = now_ms();
    publish(writer, value, "Mission.ExecutionEvent.write");
    ExecutionEventFinalize(&value);
}

class StructuredPlanner {
public:
    explicit StructuredPlanner(DdsNode& node)
        : task_writer_(dynamic_cast<TaskRequestDataWriter*>(node.writer(node.task_topic(), kMissionCommand))),
          event_writer_(dynamic_cast<ExecutionEventDataWriter*>(node.writer(node.event_topic(), kAuditEvent))) {
        if (task_writer_ == NULL || event_writer_ == NULL) {
            throw std::runtime_error("Unable to create planner DDS writers");
        }
    }

    void publish_incident_plan(const taskplan::Plan& plan) {
        for (const taskplan::Task& task : plan.tasks) {
            publish_task(task);
        }
    }

private:
    static TaskKind task_kind(const std::string& value) {
        if (value == "SURVEY") {
            return TASK_SURVEY;
        }
        if (value == "DELIVERY") {
            return TASK_DELIVERY;
        }
        throw std::runtime_error("Unsupported task kind after plan validation: " + value);
    }

    static TaskPriority task_priority(const std::string& value) {
        if (value == "NORMAL") {
            return PRIORITY_NORMAL;
        }
        if (value == "HIGH") {
            return PRIORITY_HIGH;
        }
        if (value == "CRITICAL") {
            return PRIORITY_CRITICAL;
        }
        throw std::runtime_error("Unsupported task priority after plan validation: " + value);
    }

    void publish_task(const taskplan::Task& input) {
        TaskRequest task;
        TaskRequestInitialize(&task);
        set_text(task.task_id, input.task_id);
        set_text(task.incident_id, input.incident_id);
        task.kind = task_kind(input.kind);
        task.priority = task_priority(input.priority);
        task.revision = input.revision;
        set_text(task.required_capability, input.required_capability);
        task.target_wgs84.latitude_deg = input.target_wgs84.latitude_deg;
        task.target_wgs84.longitude_deg = input.target_wgs84.longitude_deg;
        task.target_wgs84.altitude_m = input.target_wgs84.altitude_m;
        set_pose(task.target_enu, Pose{});
        set_text(task.predecessor_task_id, input.predecessor_task_id);
        task.payload_kg = input.payload_kg;
        task.deadline_s = input.deadline_s;
        set_text(task.frame_id, input.frame_id);
        set_text(task.map_version, input.map_version);
        task.created_at_ms = now_ms();
        publish(task_writer_, task, "Mission.TaskRequest.write");
        publish_event(event_writer_, "event-plan-" + input.task_id, input.incident_id, input.task_id,
            "planner-agent", EVENT_TASK_PUBLISHED, task.priority, "validated_json_plan_published");
        TaskRequestFinalize(&task);
    }

    TaskRequestDataWriter* task_writer_;
    ExecutionEventDataWriter* event_writer_;
};

class CoordinateService {
public:
    explicit CoordinateService(DdsNode& node)
        : task_reader_(dynamic_cast<TaskRequestDataReader*>(node.reader(node.task_topic(), kMissionCommand))),
          transform_writer_(dynamic_cast<CoordinateTransformDataWriter*>(node.writer(node.transform_topic(), kTelemetry))) {
        if (task_reader_ == NULL || transform_writer_ == NULL) {
            throw std::runtime_error("Unable to create coordinate service DDS endpoints");
        }
    }

    void process_tasks() {
        drain<TaskRequestDataReader, TaskRequestSeq>(task_reader_, [this](const TaskRequest& task) {
            const Pose enu = wgs84_to_enu(task.target_wgs84);
            publish_transform(text_or_empty(task.task_id), task.target_wgs84, enu);
        });
    }

private:
    static Pose wgs84_to_enu(const GeoPoint& source) {
        constexpr double pi = 3.14159265358979323846;
        const double lat0 = kReferenceLatDeg * pi / 180.0;
        const double latitude_delta = (source.latitude_deg - kReferenceLatDeg) * pi / 180.0;
        const double longitude_delta = (source.longitude_deg - kReferenceLonDeg) * pi / 180.0;
        return {
            kEarthRadiusM * longitude_delta * std::cos(lat0),
            kEarthRadiusM * latitude_delta,
            source.altitude_m,
            0.0,
        };
    }

    void publish_transform(const std::string& task_id, const GeoPoint& source, const Pose& enu) {
        CoordinateTransform value;
        CoordinateTransformInitialize(&value);
        set_text(value.transform_id, "transform-" + task_id);
        set_text(value.task_id, task_id);
        value.source_frame = FRAME_WGS84;
        value.target_frame = FRAME_CAMPUS_LOCAL;
        set_text(value.frame_id, "park_enu_v1");
        set_text(value.map_version, "campus-map-2026.1");
        value.source_wgs84 = source;
        set_pose(value.target_enu, enu);
        value.sandbox_x_px = 80.0 + enu.east_m * 2.8;
        value.sandbox_y_px = 440.0 - enu.north_m * 2.8;
        value.converted_at_ms = now_ms();
        publish(transform_writer_, value, "Mission.CoordinateTransform.write");
        CoordinateTransformFinalize(&value);
    }

    TaskRequestDataReader* task_reader_;
    CoordinateTransformDataWriter* transform_writer_;
};

class VehicleAgent {
public:
    VehicleAgent(
        DdsNode& node,
        std::string id,
        DeviceKind kind,
        std::string capability,
        double max_payload_kg,
        double speed_mps,
        double battery_percent,
        Pose home)
        : id_(std::move(id)), kind_(kind), capability_(std::move(capability)),
          max_payload_kg_(max_payload_kg), speed_mps_(speed_mps), battery_percent_(battery_percent),
          home_(home), current_pose_(home),
          task_reader_(dynamic_cast<TaskRequestDataReader*>(node.reader(node.task_topic(), kMissionCommand))),
          transform_reader_(dynamic_cast<CoordinateTransformDataReader*>(node.reader(node.transform_topic(), kTelemetry))),
          assignment_reader_(dynamic_cast<TaskAssignmentDataReader*>(node.reader(node.assignment_topic(), kMissionCommand))),
          capability_writer_(dynamic_cast<VehicleCapabilityDataWriter*>(node.writer(node.capability_topic(), kCapability))),
          bid_writer_(dynamic_cast<CandidateBidDataWriter*>(node.writer(node.bid_topic(), kBidWindow))),
          state_writer_(dynamic_cast<VehicleStateDataWriter*>(node.writer(node.state_topic(), kMissionCommand))),
          event_writer_(dynamic_cast<ExecutionEventDataWriter*>(node.writer(node.event_topic(), kAuditEvent))) {
        if (task_reader_ == NULL || transform_reader_ == NULL || assignment_reader_ == NULL ||
            capability_writer_ == NULL || bid_writer_ == NULL || state_writer_ == NULL || event_writer_ == NULL) {
            throw std::runtime_error("Unable to create vehicle DDS endpoints for " + id_);
        }
    }

    void publish_capability() {
        VehicleCapability capability;
        VehicleCapabilityInitialize(&capability);
        set_text(capability.agent_id, id_);
        capability.device_kind = kind_;
        set_text(capability.primary_capability, capability_);
        capability.max_payload_kg = max_payload_kg_;
        capability.cruise_speed_mps = speed_mps_;
        capability.battery_percent = battery_percent_;
        set_pose(capability.home_enu, home_);
        capability.available = true;
        capability.observed_at_ms = now_ms();
        publish(capability_writer_, capability, "Mission.VehicleCapability.write");
        VehicleCapabilityFinalize(&capability);
        publish_state("", PHASE_IDLE, 0.0);
    }

    void synchronize_inputs() {
        drain<TaskRequestDataReader, TaskRequestSeq>(task_reader_, [this](const TaskRequest& task) {
            tasks_[text_or_empty(task.task_id)] = as_task_info(task);
        });
        drain<CoordinateTransformDataReader, CoordinateTransformSeq>(transform_reader_, [this](const CoordinateTransform& transform) {
            targets_[text_or_empty(transform.task_id)] = as_pose(transform.target_enu);
        });
    }

    void submit_candidate_bids() {
        synchronize_inputs();
        for (const auto& entry : tasks_) {
            const TaskInfo& task = entry.second;
            if (bidded_tasks_.count(task.id) != 0 || task.capability != capability_ || task.payload_kg > max_payload_kg_) {
                continue;
            }
            const Pose target = targets_.count(task.id) != 0 ? targets_.at(task.id) : task.target;
            const double eta_s = distance_m(current_pose_, target) / speed_mps_;
            const double cost = eta_s + (100.0 - battery_percent_) * 0.35;

            CandidateBid bid;
            CandidateBidInitialize(&bid);
            const std::string bid_id = "bid-" + task.id + "-" + id_;
            set_text(bid.bid_id, bid_id);
            set_text(bid.task_id, task.id);
            set_text(bid.agent_id, id_);
            bid.feasible = true;
            bid.eta_s = eta_s;
            bid.cost_score = cost;
            bid.bid_ttl_ms = 5000;
            set_text(bid.rationale_code, "capability_match_eta_plus_battery_risk");
            bid.created_at_ms = now_ms();
            publish(bid_writer_, bid, "Mission.CandidateBid.write");
            CandidateBidFinalize(&bid);
            publish_event(event_writer_, "event-bid-" + bid_id, task.incident_id, task.id, id_,
                EVENT_BID_SUBMITTED, task.priority, "feasible_capability_bid");
            bidded_tasks_.insert(task.id);
        }
    }

    void process_assignments() {
        synchronize_inputs();
        drain<TaskAssignmentDataReader, TaskAssignmentSeq>(assignment_reader_, [this](const TaskAssignment& assignment) {
            const std::string task_id = text_or_empty(assignment.task_id);
            const auto seen_epoch = assignment_epochs_.find(task_id);
            if (!assignment.accepted || text_or_empty(assignment.selected_agent_id) != id_ ||
                executed_tasks_.count(task_id) != 0 ||
                (seen_epoch != assignment_epochs_.end() && seen_epoch->second >= assignment.assignment_epoch)) {
                return;
            }
            assignment_epochs_[task_id] = assignment.assignment_epoch;
            const auto task = tasks_.find(task_id);
            if (task == tasks_.end()) {
                return;
            }
            const Pose target = targets_.count(task_id) != 0 ? targets_.at(task_id) : task->second.target;
            execute(task->second, target);
            executed_tasks_.insert(task_id);
        });
    }

private:
    void publish_state(const std::string& task_id, VehiclePhase phase, double progress_percent) {
        VehicleState state;
        VehicleStateInitialize(&state);
        set_text(state.agent_id, id_);
        set_text(state.active_task_id, task_id);
        state.device_kind = kind_;
        state.phase = phase;
        set_pose(state.pose_enu, current_pose_);
        set_text(state.frame_id, "park_enu_v1");
        set_text(state.map_version, "campus-map-2026.1");
        state.battery_percent = battery_percent_;
        state.progress_percent = progress_percent;
        state.updated_at_ms = now_ms();
        publish(state_writer_, state, "Mission.VehicleState.write");
        VehicleStateFinalize(&state);
    }

    void execute(const TaskInfo& task, const Pose& target) {
        publish_state(task.id, PHASE_ASSIGNED, 0.0);
        publish_event(event_writer_, "event-assigned-" + task.id + "-" + id_, task.incident_id, task.id,
            id_, EVENT_TASK_ASSIGNED, task.priority, "assignment_accepted");
        settle(120);
        current_pose_ = lerp(current_pose_, target, 0.45);
        battery_percent_ -= 1.0;
        publish_state(task.id, PHASE_EN_ROUTE, 45.0);
        publish_event(event_writer_, "event-progress-1-" + task.id, task.incident_id, task.id,
            id_, EVENT_PROGRESS, task.priority, "route_progress_45_percent");
        settle(120);
        current_pose_ = target;
        battery_percent_ -= 1.2;
        publish_state(task.id, PHASE_EXECUTING, 85.0);
        publish_event(event_writer_, "event-progress-2-" + task.id, task.incident_id, task.id,
            id_, EVENT_PROGRESS, task.priority, "on_target_executing");
        settle(120);
        battery_percent_ -= 0.5;
        publish_state(task.id, PHASE_COMPLETED, 100.0);
        publish_event(event_writer_, "event-completed-" + task.id, task.incident_id, task.id,
            id_, EVENT_TASK_COMPLETED, task.priority, "task_completed");
    }

    std::string id_;
    DeviceKind kind_;
    std::string capability_;
    double max_payload_kg_;
    double speed_mps_;
    double battery_percent_;
    Pose home_;
    Pose current_pose_;
    TaskRequestDataReader* task_reader_;
    CoordinateTransformDataReader* transform_reader_;
    TaskAssignmentDataReader* assignment_reader_;
    VehicleCapabilityDataWriter* capability_writer_;
    CandidateBidDataWriter* bid_writer_;
    VehicleStateDataWriter* state_writer_;
    ExecutionEventDataWriter* event_writer_;
    std::map<std::string, TaskInfo> tasks_;
    std::map<std::string, Pose> targets_;
    std::set<std::string> bidded_tasks_;
    std::set<std::string> executed_tasks_;
    std::map<std::string, DDS_Long> assignment_epochs_;
};

class Coordinator {
public:
    explicit Coordinator(DdsNode& node)
        : task_reader_(dynamic_cast<TaskRequestDataReader*>(node.reader(node.task_topic(), kMissionCommand))),
          capability_reader_(dynamic_cast<VehicleCapabilityDataReader*>(node.reader(node.capability_topic(), kCapability))),
          bid_reader_(dynamic_cast<CandidateBidDataReader*>(node.reader(node.bid_topic(), kBidWindow))),
          state_reader_(dynamic_cast<VehicleStateDataReader*>(node.reader(node.state_topic(), kMissionCommand))),
          transform_reader_(dynamic_cast<CoordinateTransformDataReader*>(node.reader(node.transform_topic(), kTelemetry))),
          assignment_writer_(dynamic_cast<TaskAssignmentDataWriter*>(node.writer(node.assignment_topic(), kMissionCommand))),
          event_writer_(dynamic_cast<ExecutionEventDataWriter*>(node.writer(node.event_topic(), kAuditEvent))) {
        if (task_reader_ == NULL || capability_reader_ == NULL || bid_reader_ == NULL || state_reader_ == NULL ||
            transform_reader_ == NULL || assignment_writer_ == NULL || event_writer_ == NULL) {
            throw std::runtime_error("Unable to create coordinator DDS endpoints");
        }
    }

    void synchronize_inputs() {
        drain<TaskRequestDataReader, TaskRequestSeq>(task_reader_, [this](const TaskRequest& task) {
            tasks_[text_or_empty(task.task_id)] = as_task_info(task);
        });
        drain<VehicleCapabilityDataReader, VehicleCapabilitySeq>(capability_reader_, [this](const VehicleCapability& capability) {
            capabilities_[text_or_empty(capability.agent_id)] = as_capability_info(capability);
        });
        drain<CandidateBidDataReader, CandidateBidSeq>(bid_reader_, [this](const CandidateBid& bid) {
            bids_.push_back(as_bid_info(bid));
        });
        drain<VehicleStateDataReader, VehicleStateSeq>(state_reader_, [this](const VehicleState& state) {
            if (state.phase == PHASE_COMPLETED && text_or_empty(state.active_task_id).size() > 0) {
                completed_tasks_.insert(text_or_empty(state.active_task_id));
            }
        });
        drain<CoordinateTransformDataReader, CoordinateTransformSeq>(transform_reader_, [this](const CoordinateTransform& transform) {
            targets_[text_or_empty(transform.task_id)] = as_pose(transform.target_enu);
        });
    }

    void schedule_ready_tasks() {
        synchronize_inputs();
        for (const auto& entry : tasks_) {
            const TaskInfo& task = entry.second;
            if (assigned_tasks_.count(task.id) != 0 ||
                (!task.predecessor.empty() && completed_tasks_.count(task.predecessor) == 0)) {
                continue;
            }
            const BidInfo* winner = choose_candidate(task);
            if (winner == NULL) {
                continue;
            }
            publish_assignment(task, *winner);
            assigned_tasks_.insert(task.id);
        }
    }

private:
    const BidInfo* choose_candidate(const TaskInfo& task) const {
        const BidInfo* winner = NULL;
        for (const BidInfo& bid : bids_) {
            if (!bid.feasible || bid.task_id != task.id ||
                now_ms() - bid.created_at_ms > bid.bid_ttl_ms) {
                continue;
            }
            const auto capability = capabilities_.find(bid.agent_id);
            if (capability == capabilities_.end() || !capability->second.available ||
                capability->second.capability != task.capability) {
                continue;
            }
            if (winner == NULL || bid.cost < winner->cost ||
                (bid.cost == winner->cost && bid.eta_s < winner->eta_s) ||
                (bid.cost == winner->cost && bid.eta_s == winner->eta_s && bid.id < winner->id)) {
                winner = &bid;
            }
        }
        return winner;
    }

    void publish_assignment(const TaskInfo& task, const BidInfo& bid) {
        TaskAssignment assignment;
        TaskAssignmentInitialize(&assignment);
        set_text(assignment.task_id, task.id);
        set_text(assignment.incident_id, task.incident_id);
        set_text(assignment.selected_agent_id, bid.agent_id);
        set_text(assignment.selected_bid_id, bid.id);
        assignment.expected_eta_s = bid.eta_s;
        assignment.assignment_epoch = static_cast<DDS_Long>(assigned_tasks_.size() + 1);
        set_text(assignment.dispatcher_boot_id, "dispatcher-demo-001");
        assignment.accepted = true;
        assignment.assigned_at_ms = now_ms();
        publish(assignment_writer_, assignment, "Mission.TaskAssignment.write");
        TaskAssignmentFinalize(&assignment);
        publish_event(event_writer_, "event-decision-" + task.id, task.incident_id, task.id,
            "coordinator-agent", EVENT_TASK_ASSIGNED, task.priority,
            "lowest_cost_feasible_bid_selected");
    }

    TaskRequestDataReader* task_reader_;
    VehicleCapabilityDataReader* capability_reader_;
    CandidateBidDataReader* bid_reader_;
    VehicleStateDataReader* state_reader_;
    CoordinateTransformDataReader* transform_reader_;
    TaskAssignmentDataWriter* assignment_writer_;
    ExecutionEventDataWriter* event_writer_;
    std::map<std::string, TaskInfo> tasks_;
    std::map<std::string, CapabilityInfo> capabilities_;
    std::map<std::string, Pose> targets_;
    std::vector<BidInfo> bids_;
    std::set<std::string> completed_tasks_;
    std::set<std::string> assigned_tasks_;
};

struct DashboardEvent {
    std::string id;
    std::string task_id;
    std::string source;
    EventKind kind = EVENT_PROGRESS;
    std::string code;
    long long occurred_at_ms = 0;
};

struct DashboardState {
    std::string agent_id;
    std::string task_id;
    DeviceKind kind = DEVICE_UAV;
    VehiclePhase phase = PHASE_IDLE;
    Pose pose;
    double battery_percent = 0.0;
    double progress_percent = 0.0;
};

class DashboardBridge {
public:
    explicit DashboardBridge(DdsNode& node)
        : task_reader_(dynamic_cast<TaskRequestDataReader*>(node.reader(node.task_topic(), kMissionCommand))),
          assignment_reader_(dynamic_cast<TaskAssignmentDataReader*>(node.reader(node.assignment_topic(), kMissionCommand))),
          state_reader_(dynamic_cast<VehicleStateDataReader*>(node.reader(node.state_topic(), kMissionCommand))),
          event_reader_(dynamic_cast<ExecutionEventDataReader*>(node.reader(node.event_topic(), kAuditEvent))),
          transform_reader_(dynamic_cast<CoordinateTransformDataReader*>(node.reader(node.transform_topic(), kTelemetry))) {
        if (task_reader_ == NULL || assignment_reader_ == NULL || state_reader_ == NULL ||
            event_reader_ == NULL || transform_reader_ == NULL) {
            throw std::runtime_error("Unable to create dashboard DDS readers");
        }
    }

    void collect() {
        drain<TaskRequestDataReader, TaskRequestSeq>(task_reader_, [this](const TaskRequest& task) {
            tasks_[text_or_empty(task.task_id)] = as_task_info(task);
        });
        drain<TaskAssignmentDataReader, TaskAssignmentSeq>(assignment_reader_, [this](const TaskAssignment& assignment) {
            assignments_[text_or_empty(assignment.task_id)] = text_or_empty(assignment.selected_agent_id);
        });
        drain<VehicleStateDataReader, VehicleStateSeq>(state_reader_, [this](const VehicleState& state) {
            DashboardState snapshot;
            snapshot.agent_id = text_or_empty(state.agent_id);
            snapshot.task_id = text_or_empty(state.active_task_id);
            snapshot.kind = state.device_kind;
            snapshot.phase = state.phase;
            snapshot.pose = as_pose(state.pose_enu);
            snapshot.battery_percent = state.battery_percent;
            snapshot.progress_percent = state.progress_percent;
            states_[snapshot.agent_id] = snapshot;
        });
        drain<ExecutionEventDataReader, ExecutionEventSeq>(event_reader_, [this](const ExecutionEvent& event) {
            events_.push_back({
                text_or_empty(event.event_id), text_or_empty(event.task_id),
                text_or_empty(event.source_agent_id), event.kind,
                text_or_empty(event.message_code), event.occurred_at_ms});
        });
        drain<CoordinateTransformDataReader, CoordinateTransformSeq>(transform_reader_, [this](const CoordinateTransform& transform) {
            transforms_[text_or_empty(transform.task_id)] = transform_to_json_data(transform);
        });
        std::sort(events_.begin(), events_.end(), [](const DashboardEvent& left, const DashboardEvent& right) {
            return left.occurred_at_ms < right.occurred_at_ms;
        });
    }

    bool write_snapshot(const std::filesystem::path& output_path) const {
        std::filesystem::create_directories(output_path.parent_path());
        std::ofstream out(output_path, std::ios::trunc);
        if (!out) {
            return false;
        }
        out << "{\n";
        out << "  \"scenario\": \"campus emergency response\",\n";
        out << "  \"domain_id\": " << kDomainId << ",\n";
        out << "  \"transport\": \"ZRDDS typed topics via DashboardBridge\",\n";
        out << "  \"generated_at_ms\": " << now_ms() << ",\n";
        out << "  \"tasks\": [\n";
        bool first = true;
        for (const auto& entry : tasks_) {
            if (!first) { out << ",\n"; }
            first = false;
            const TaskInfo& task = entry.second;
            const auto transform = transforms_.find(task.id);
            const auto assignment = assignments_.find(task.id);
            out << "    {\"id\":\"" << escape(task.id) << "\",\"kind\":\"" << task_kind_name(task.kind)
                << "\",\"priority\":\"" << priority_name(task.priority) << "\",\"capability\":\""
                << escape(task.capability) << "\",\"predecessor\":\"" << escape(task.predecessor)
                << "\",\"assigned_to\":\"" << (assignment == assignments_.end() ? "" : escape(assignment->second))
                << "\",\"enu\":{\"east\":" << (transform == transforms_.end() ? task.target.east_m : transform->second.enu.east_m)
                << ",\"north\":" << (transform == transforms_.end() ? task.target.north_m : transform->second.enu.north_m)
                << ",\"up\":" << (transform == transforms_.end() ? task.target.up_m : transform->second.enu.up_m)
                << "},\"sandbox\":{\"x\":" << (transform == transforms_.end() ? 0.0 : transform->second.sandbox_x_px)
                << ",\"y\":" << (transform == transforms_.end() ? 0.0 : transform->second.sandbox_y_px) << "}}";
        }
        out << "\n  ],\n";
        out << "  \"vehicles\": [\n";
        first = true;
        for (const auto& entry : states_) {
            if (!first) { out << ",\n"; }
            first = false;
            const DashboardState& state = entry.second;
            out << "    {\"id\":\"" << escape(state.agent_id) << "\",\"kind\":\"" << device_name(state.kind)
                << "\",\"phase\":\"" << phase_name(state.phase) << "\",\"task_id\":\"" << escape(state.task_id)
                << "\",\"battery_percent\":" << state.battery_percent << ",\"progress_percent\":" << state.progress_percent
                << ",\"enu\":{\"east\":" << state.pose.east_m << ",\"north\":" << state.pose.north_m
                << ",\"up\":" << state.pose.up_m << "}}";
        }
        out << "\n  ],\n";
        out << "  \"events\": [\n";
        first = true;
        for (const DashboardEvent& event : events_) {
            if (!first) { out << ",\n"; }
            first = false;
            out << "    {\"id\":\"" << escape(event.id) << "\",\"task_id\":\"" << escape(event.task_id)
                << "\",\"source\":\"" << escape(event.source) << "\",\"kind\":\"" << event_name(event.kind)
                << "\",\"code\":\"" << escape(event.code) << "\",\"occurred_at_ms\":" << event.occurred_at_ms << "}";
        }
        out << "\n  ]\n}\n";
        return true;
    }

    std::size_t completed_task_count() const {
        std::set<std::string> completed;
        for (const DashboardEvent& event : events_) {
            if (event.kind == EVENT_TASK_COMPLETED) {
                completed.insert(event.task_id);
            }
        }
        return completed.size();
    }

private:
    struct TransformData {
        Pose enu;
        double sandbox_x_px = 0.0;
        double sandbox_y_px = 0.0;
    };

    static TransformData transform_to_json_data(const CoordinateTransform& value) {
        return {as_pose(value.target_enu), value.sandbox_x_px, value.sandbox_y_px};
    }

    static std::string escape(const std::string& value) {
        std::string result;
        for (const char character : value) {
            switch (character) {
                case '\\': result += "\\\\"; break;
                case '\"': result += "\\\""; break;
                case '\n': result += "\\n"; break;
                case '\r': result += "\\r"; break;
                case '\t': result += "\\t"; break;
                default: result += character; break;
            }
        }
        return result;
    }

    static const char* task_kind_name(TaskKind kind) {
        return kind == TASK_SURVEY ? "SURVEY" : "DELIVERY";
    }

    static const char* priority_name(TaskPriority priority) {
        switch (priority) {
            case PRIORITY_CRITICAL: return "CRITICAL";
            case PRIORITY_HIGH: return "HIGH";
            default: return "NORMAL";
        }
    }

    static const char* device_name(DeviceKind kind) {
        return kind == DEVICE_UAV ? "UAV" : "UGV";
    }

    static const char* phase_name(VehiclePhase phase) {
        switch (phase) {
            case PHASE_IDLE: return "IDLE";
            case PHASE_BIDDING: return "BIDDING";
            case PHASE_ASSIGNED: return "ASSIGNED";
            case PHASE_EN_ROUTE: return "EN_ROUTE";
            case PHASE_EXECUTING: return "EXECUTING";
            case PHASE_COMPLETED: return "COMPLETED";
            default: return "FAILED";
        }
    }

    static const char* event_name(EventKind kind) {
        switch (kind) {
            case EVENT_TASK_PUBLISHED: return "TASK_PUBLISHED";
            case EVENT_BID_SUBMITTED: return "BID_SUBMITTED";
            case EVENT_TASK_ASSIGNED: return "TASK_ASSIGNED";
            case EVENT_PROGRESS: return "PROGRESS";
            case EVENT_TASK_COMPLETED: return "TASK_COMPLETED";
            default: return "SAFETY_ALERT";
        }
    }

    TaskRequestDataReader* task_reader_;
    TaskAssignmentDataReader* assignment_reader_;
    VehicleStateDataReader* state_reader_;
    ExecutionEventDataReader* event_reader_;
    CoordinateTransformDataReader* transform_reader_;
    std::map<std::string, TaskInfo> tasks_;
    std::map<std::string, std::string> assignments_;
    std::map<std::string, DashboardState> states_;
    std::map<std::string, TransformData> transforms_;
    std::vector<DashboardEvent> events_;
};

std::filesystem::path path_from_args(
    int argc,
    char** argv,
    const std::string& option,
    const std::filesystem::path& default_value) {
    for (int index = 1; index < argc; ++index) {
        if (std::string(argv[index]) == option) {
            if (index + 1 >= argc) {
                throw std::runtime_error("Missing path after " + option);
            }
            return argv[index + 1];
        }
    }
    return default_value;
}

std::filesystem::path dashboard_path_from_args(int argc, char** argv) {
    return path_from_args(argc, argv, "--dashboard", "dashboard/telemetry.json");
}

std::filesystem::path task_plan_path_from_args(int argc, char** argv) {
    return path_from_args(argc, argv, "--task-plan", "task_plan.json");
}

void finalize_type_supports() {
    TaskRequestTypeSupport::finalize_instance();
    VehicleCapabilityTypeSupport::finalize_instance();
    CandidateBidTypeSupport::finalize_instance();
    TaskAssignmentTypeSupport::finalize_instance();
    VehicleStateTypeSupport::finalize_instance();
    ExecutionEventTypeSupport::finalize_instance();
    CoordinateTransformTypeSupport::finalize_instance();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path plan_path = task_plan_path_from_args(argc, argv);
        const taskplan::Plan plan = taskplan::load_file(plan_path);
        {
            DdsNode planner_node("planner-agent");
            DdsNode coordinate_node("coordinate-service");
            DdsNode uav_node("uav-agent");
            DdsNode ugv_node("ugv-agent");
            DdsNode coordinator_node("coordinator-agent");
            DdsNode dashboard_node("dashboard-bridge");

            StructuredPlanner planner(planner_node);
            CoordinateService coordinates(coordinate_node);
            VehicleAgent uav(uav_node, "uav-alpha", DEVICE_UAV, "CAMERA_THERMAL", 1.0, 18.0, 94.0, {0.0, -40.0, 20.0, 0.0});
            VehicleAgent ugv(ugv_node, "ugv-bravo", DEVICE_UGV, "MEDICAL_PAYLOAD", 12.0, 5.0, 88.0, {220.0, -160.0, 0.0, 90.0});
            Coordinator coordinator(coordinator_node);
            DashboardBridge dashboard(dashboard_node);

            // All participants exist before publishing so the demo has no start-order dependency.
            settle(500);
            uav.publish_capability();
            ugv.publish_capability();
            planner.publish_incident_plan(plan);
            settle();
            coordinates.process_tasks();
            settle();
            uav.submit_candidate_bids();
            ugv.submit_candidate_bids();
            settle();

            coordinator.schedule_ready_tasks(); // Survey is feasible; delivery waits for its predecessor.
            settle();
            uav.process_assignments();
            settle();
            coordinator.schedule_ready_tasks(); // UAV completion unlocks medical delivery.
            settle();
            ugv.process_assignments();
            settle(300);

            dashboard.collect();
            const std::filesystem::path output = dashboard_path_from_args(argc, argv);
            if (!dashboard.write_snapshot(output)) {
                throw std::runtime_error("Unable to write dashboard snapshot: " + output.string());
            }
            if (dashboard.completed_task_count() != plan.tasks.size()) {
                throw std::runtime_error("Demo invariant failed: both dependent tasks were not completed");
            }

            std::cout << "ZRDDS demo completed: " << plan.tasks.size()
                      << " dependent tasks dispatched through typed DDS topics.\n";
            std::cout << "Task plan: " << std::filesystem::absolute(plan_path).string() << "\n";
            std::cout << "Dashboard snapshot: " << std::filesystem::absolute(output).string() << "\n";
        }

        finalize_type_supports();
        DDS::DomainParticipantFactory::get_instance()->finalize_instance();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "scheduler_demo failed: " << error.what() << "\n";
        return 1;
    }
}
