#include "task_plan.h"

#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: task_plan_parser_test <task_plan.example.json>\n";
        return 2;
    }

    try {
        const taskplan::Plan plan = taskplan::load_file(argv[1]);
        if (plan.tasks.size() != 2 || plan.tasks[0].kind != "SURVEY" || plan.tasks[1].kind != "DELIVERY") {
            throw std::runtime_error("example plan was not mapped correctly");
        }

        bool rejected = false;
        try {
            (void)taskplan::parse_json("{\"tasks\":[]}");
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        if (!rejected) {
            throw std::runtime_error("invalid plan was accepted");
        }

        rejected = false;
        try {
            (void)taskplan::parse_json("{\"tasks\":[],\"tasks\":[]}");
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        if (!rejected) {
            throw std::runtime_error("duplicate JSON property was accepted");
        }

        rejected = false;
        try {
            std::string deeply_nested(70, '[');
            deeply_nested += "0";
            deeply_nested += std::string(70, ']');
            (void)taskplan::parse_json(deeply_nested);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        if (!rejected) {
            throw std::runtime_error("excessively nested JSON was accepted");
        }

        std::string wrong_frame =
            std::string("{\"tasks\":[") +
            "{\"task_id\":\"s\",\"incident_id\":\"i\",\"kind\":\"SURVEY\","
            "\"priority\":\"NORMAL\",\"revision\":1,\"required_capability\":\"CAMERA_THERMAL\","
            "\"target_wgs84\":{\"latitude_deg\":0,\"longitude_deg\":0,\"altitude_m\":0},"
            "\"predecessor_task_id\":\"\",\"payload_kg\":0,\"deadline_s\":1,"
            "\"frame_id\":\"wrong\",\"map_version\":\"campus-map-2026.1\"},"
            "{\"task_id\":\"d\",\"incident_id\":\"i\",\"kind\":\"DELIVERY\","
            "\"priority\":\"NORMAL\",\"revision\":1,\"required_capability\":\"MEDICAL_PAYLOAD\","
            "\"target_wgs84\":{\"latitude_deg\":0,\"longitude_deg\":0,\"altitude_m\":0},"
            "\"predecessor_task_id\":\"s\",\"payload_kg\":1,\"deadline_s\":1,"
            "\"frame_id\":\"wrong\",\"map_version\":\"campus-map-2026.1\"}]}";
        rejected = false;
        try {
            (void)taskplan::parse_json(wrong_frame);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        if (!rejected) {
            throw std::runtime_error("non-canonical coordinate metadata was accepted");
        }

        std::cout << "C++ task-plan parser tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "C++ task-plan parser test failed: " << error.what() << "\n";
        return 1;
    }
}
