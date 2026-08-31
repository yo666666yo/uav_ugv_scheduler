#include "task_plan.h"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace taskplan {
namespace {

enum class JsonType { Null, Boolean, Number, String, Array, Object };

struct JsonValue {
    JsonType type = JsonType::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<JsonValue> array;
    std::map<std::string, JsonValue> object;
};

class JsonParser {
public:
    explicit JsonParser(const std::string& input) : input_(input) {}

    JsonValue parse() {
        JsonValue result = parse_value();
        skip_whitespace();
        if (position_ != input_.size()) {
            fail("unexpected content after the JSON value");
        }
        return result;
    }

private:
    [[noreturn]] void fail(const std::string& message) const {
        std::ostringstream out;
        out << "Invalid task plan JSON at byte " << position_ << ": " << message;
        throw std::runtime_error(out.str());
    }

    void skip_whitespace() {
        while (position_ < input_.size() &&
               (input_[position_] == ' ' || input_[position_] == '\n' ||
                input_[position_] == '\r' || input_[position_] == '\t')) {
            ++position_;
        }
    }

    bool consume(char expected) {
        skip_whitespace();
        if (position_ < input_.size() && input_[position_] == expected) {
            ++position_;
            return true;
        }
        return false;
    }

    void expect(char expected) {
        if (!consume(expected)) {
            fail(std::string("expected '") + expected + "'");
        }
    }

    JsonValue parse_value() {
        skip_whitespace();
        if (position_ >= input_.size()) {
            fail("expected a JSON value");
        }
        switch (input_[position_]) {
            case '{': return parse_object();
            case '[': return parse_array();
            case '"': {
                JsonValue value;
                value.type = JsonType::String;
                value.string = parse_string();
                return value;
            }
            case 't': return parse_literal("true", JsonType::Boolean, true);
            case 'f': return parse_literal("false", JsonType::Boolean, false);
            case 'n': return parse_literal("null", JsonType::Null, false);
            default:
                if (input_[position_] == '-' || std::isdigit(static_cast<unsigned char>(input_[position_]))) {
                    return parse_number();
                }
                fail("unexpected character");
        }
    }

    JsonValue parse_object() {
        expect('{');
        JsonValue value;
        value.type = JsonType::Object;
        if (consume('}')) {
            return value;
        }
        while (true) {
            skip_whitespace();
            if (position_ >= input_.size() || input_[position_] != '"') {
                fail("expected an object property name");
            }
            std::string key = parse_string();
            expect(':');
            const auto inserted = value.object.emplace(key, parse_value());
            if (!inserted.second) {
                fail("duplicate object property: " + key);
            }
            if (consume('}')) {
                return value;
            }
            expect(',');
        }
    }

    JsonValue parse_array() {
        expect('[');
        JsonValue value;
        value.type = JsonType::Array;
        if (consume(']')) {
            return value;
        }
        while (true) {
            value.array.push_back(parse_value());
            if (consume(']')) {
                return value;
            }
            expect(',');
        }
    }

    static void append_utf8(std::string& output, std::uint32_t codepoint) {
        if (codepoint <= 0x7f) {
            output.push_back(static_cast<char>(codepoint));
        } else if (codepoint <= 0x7ff) {
            output.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
            output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
        } else if (codepoint <= 0xffff) {
            output.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
            output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
            output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
        } else {
            output.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
            output.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
            output.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
            output.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
        }
    }

    std::uint32_t parse_hex_quad() {
        if (position_ + 4 > input_.size()) {
            fail("incomplete Unicode escape");
        }
        std::uint32_t value = 0;
        for (int index = 0; index < 4; ++index) {
            const char character = input_[position_++];
            value <<= 4;
            if (character >= '0' && character <= '9') {
                value += static_cast<std::uint32_t>(character - '0');
            } else if (character >= 'a' && character <= 'f') {
                value += static_cast<std::uint32_t>(character - 'a' + 10);
            } else if (character >= 'A' && character <= 'F') {
                value += static_cast<std::uint32_t>(character - 'A' + 10);
            } else {
                fail("invalid Unicode escape");
            }
        }
        return value;
    }

    std::string parse_string() {
        if (position_ >= input_.size() || input_[position_] != '"') {
            fail("expected a string");
        }
        ++position_;
        std::string result;
        while (position_ < input_.size()) {
            const unsigned char character = static_cast<unsigned char>(input_[position_++]);
            if (character == '"') {
                return result;
            }
            if (character < 0x20) {
                fail("unescaped control character in string");
            }
            if (character != '\\') {
                result.push_back(static_cast<char>(character));
                continue;
            }
            if (position_ >= input_.size()) {
                fail("incomplete string escape");
            }
            const char escaped = input_[position_++];
            switch (escaped) {
                case '"': result.push_back('"'); break;
                case '\\': result.push_back('\\'); break;
                case '/': result.push_back('/'); break;
                case 'b': result.push_back('\b'); break;
                case 'f': result.push_back('\f'); break;
                case 'n': result.push_back('\n'); break;
                case 'r': result.push_back('\r'); break;
                case 't': result.push_back('\t'); break;
                case 'u': {
                    std::uint32_t codepoint = parse_hex_quad();
                    if (codepoint >= 0xd800 && codepoint <= 0xdbff) {
                        if (position_ + 2 > input_.size() || input_[position_] != '\\' || input_[position_ + 1] != 'u') {
                            fail("high surrogate is not followed by a low surrogate");
                        }
                        position_ += 2;
                        const std::uint32_t low = parse_hex_quad();
                        if (low < 0xdc00 || low > 0xdfff) {
                            fail("invalid low surrogate");
                        }
                        codepoint = 0x10000 + ((codepoint - 0xd800) << 10) + (low - 0xdc00);
                    } else if (codepoint >= 0xdc00 && codepoint <= 0xdfff) {
                        fail("unexpected low surrogate");
                    }
                    append_utf8(result, codepoint);
                    break;
                }
                default: fail("invalid string escape");
            }
        }
        fail("unterminated string");
    }

    JsonValue parse_number() {
        const std::size_t start = position_;
        if (input_[position_] == '-') {
            ++position_;
        }
        if (position_ >= input_.size()) {
            fail("incomplete number");
        }
        if (input_[position_] == '0') {
            ++position_;
            if (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
                fail("leading zero in number");
            }
        } else if (input_[position_] >= '1' && input_[position_] <= '9') {
            while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
                ++position_;
            }
        } else {
            fail("invalid number");
        }
        if (position_ < input_.size() && input_[position_] == '.') {
            ++position_;
            if (position_ >= input_.size() || !std::isdigit(static_cast<unsigned char>(input_[position_]))) {
                fail("invalid fractional number");
            }
            while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
                ++position_;
            }
        }
        if (position_ < input_.size() && (input_[position_] == 'e' || input_[position_] == 'E')) {
            ++position_;
            if (position_ < input_.size() && (input_[position_] == '+' || input_[position_] == '-')) {
                ++position_;
            }
            if (position_ >= input_.size() || !std::isdigit(static_cast<unsigned char>(input_[position_]))) {
                fail("invalid number exponent");
            }
            while (position_ < input_.size() && std::isdigit(static_cast<unsigned char>(input_[position_]))) {
                ++position_;
            }
        }
        JsonValue value;
        value.type = JsonType::Number;
        try {
            value.number = std::stod(input_.substr(start, position_ - start));
        } catch (const std::exception&) {
            fail("number is outside the supported range");
        }
        if (!std::isfinite(value.number)) {
            fail("number must be finite");
        }
        return value;
    }

    JsonValue parse_literal(const char* literal, JsonType type, bool boolean) {
        const std::string text(literal);
        if (input_.compare(position_, text.size(), text) != 0) {
            fail("invalid literal");
        }
        position_ += text.size();
        JsonValue value;
        value.type = type;
        value.boolean = boolean;
        return value;
    }

    const std::string& input_;
    std::size_t position_ = 0;
};

const JsonValue& require_type(const JsonValue& value, JsonType type, const std::string& path) {
    if (value.type != type) {
        throw std::runtime_error("Invalid task plan: " + path + " has the wrong JSON type");
    }
    return value;
}

const JsonValue& require_property(const JsonValue& object, const std::string& name, const std::string& path) {
    require_type(object, JsonType::Object, path);
    const auto found = object.object.find(name);
    if (found == object.object.end()) {
        throw std::runtime_error("Invalid task plan: missing " + path + "." + name);
    }
    return found->second;
}

void require_exact_properties(
    const JsonValue& value,
    const std::set<std::string>& expected,
    const std::string& path) {
    require_type(value, JsonType::Object, path);
    std::set<std::string> actual;
    for (const auto& property : value.object) {
        actual.insert(property.first);
    }
    if (actual != expected) {
        throw std::runtime_error("Invalid task plan: " + path + " has missing or unsupported properties");
    }
}

std::string require_string(
    const JsonValue& object,
    const std::string& name,
    const std::string& path,
    std::size_t max_length,
    bool allow_empty = false) {
    const JsonValue& value = require_type(require_property(object, name, path), JsonType::String, path + "." + name);
    if ((!allow_empty && value.string.empty()) || value.string.size() > max_length) {
        throw std::runtime_error("Invalid task plan: " + path + "." + name + " has an invalid length");
    }
    return value.string;
}

double require_number(
    const JsonValue& object,
    const std::string& name,
    const std::string& path,
    double minimum,
    double maximum) {
    const JsonValue& value = require_type(require_property(object, name, path), JsonType::Number, path + "." + name);
    if (value.number < minimum || value.number > maximum) {
        throw std::runtime_error("Invalid task plan: " + path + "." + name + " is outside the allowed range");
    }
    return value.number;
}

long require_integer(
    const JsonValue& object,
    const std::string& name,
    const std::string& path,
    long minimum,
    long maximum) {
    const double number = require_number(object, name, path, static_cast<double>(minimum), static_cast<double>(maximum));
    if (std::floor(number) != number) {
        throw std::runtime_error("Invalid task plan: " + path + "." + name + " must be an integer");
    }
    return static_cast<long>(number);
}

bool is_safe_identifier(const std::string& value) {
    if (value.empty() || !std::isalnum(static_cast<unsigned char>(value.front()))) {
        return false;
    }
    for (const unsigned char character : value) {
        if (!std::isalnum(character) && character != '.' && character != '_' && character != '-') {
            return false;
        }
    }
    return true;
}

void require_safe_identifier(const std::string& value, const std::string& path) {
    if (!is_safe_identifier(value)) {
        throw std::runtime_error("Invalid task plan: " + path + " must use only letters, digits, '.', '_' or '-'");
    }
}

Task parse_task(const JsonValue& value, std::size_t index) {
    const std::string path = "tasks[" + std::to_string(index) + "]";
    require_exact_properties(value, {
        "task_id", "incident_id", "kind", "priority", "revision", "required_capability",
        "target_wgs84", "predecessor_task_id", "payload_kg", "deadline_s", "frame_id", "map_version"
    }, path);

    Task task;
    task.task_id = require_string(value, "task_id", path, 64);
    task.incident_id = require_string(value, "incident_id", path, 64);
    task.kind = require_string(value, "kind", path, 16);
    task.priority = require_string(value, "priority", path, 16);
    task.revision = require_integer(value, "revision", path, 1, 2147483647L);
    task.required_capability = require_string(value, "required_capability", path, 48);
    task.predecessor_task_id = require_string(value, "predecessor_task_id", path, 64, true);
    task.payload_kg = require_number(value, "payload_kg", path, 0.0, 12.0);
    task.deadline_s = require_integer(value, "deadline_s", path, 1, 86400);
    task.frame_id = require_string(value, "frame_id", path, 32);
    task.map_version = require_string(value, "map_version", path, 32);

    require_safe_identifier(task.task_id, path + ".task_id");
    require_safe_identifier(task.incident_id, path + ".incident_id");
    require_safe_identifier(task.frame_id, path + ".frame_id");
    require_safe_identifier(task.map_version, path + ".map_version");
    if (!task.predecessor_task_id.empty()) {
        require_safe_identifier(task.predecessor_task_id, path + ".predecessor_task_id");
    }

    const JsonValue& target = require_property(value, "target_wgs84", path);
    require_exact_properties(target, {"latitude_deg", "longitude_deg", "altitude_m"}, path + ".target_wgs84");
    task.target_wgs84.latitude_deg = require_number(target, "latitude_deg", path + ".target_wgs84", -90.0, 90.0);
    task.target_wgs84.longitude_deg = require_number(target, "longitude_deg", path + ".target_wgs84", -180.0, 180.0);
    task.target_wgs84.altitude_m = require_number(target, "altitude_m", path + ".target_wgs84", -500.0, 10000.0);

    if (task.priority != "NORMAL" && task.priority != "HIGH" && task.priority != "CRITICAL") {
        throw std::runtime_error("Invalid task plan: " + path + ".priority is unsupported");
    }
    if (task.kind == "SURVEY") {
        if (task.required_capability != "CAMERA_THERMAL" || task.payload_kg != 0.0) {
            throw std::runtime_error("Invalid task plan: SURVEY requires CAMERA_THERMAL and payload_kg 0");
        }
    } else if (task.kind == "DELIVERY") {
        if (task.required_capability != "MEDICAL_PAYLOAD" || task.payload_kg <= 0.0) {
            throw std::runtime_error("Invalid task plan: DELIVERY requires MEDICAL_PAYLOAD and payload_kg greater than 0");
        }
    } else {
        throw std::runtime_error("Invalid task plan: " + path + ".kind is unsupported");
    }
    return task;
}

void validate_plan(const Plan& plan) {
    if (plan.tasks.size() != 2) {
        throw std::runtime_error("Invalid task plan: this demo requires exactly one SURVEY and one DELIVERY task");
    }
    const Task* survey = nullptr;
    const Task* delivery = nullptr;
    std::set<std::string> task_ids;
    for (const Task& task : plan.tasks) {
        if (!task_ids.insert(task.task_id).second) {
            throw std::runtime_error("Invalid task plan: duplicate task_id " + task.task_id);
        }
        if (task.kind == "SURVEY") {
            if (survey != nullptr) {
                throw std::runtime_error("Invalid task plan: multiple SURVEY tasks are not supported");
            }
            survey = &task;
        } else {
            if (delivery != nullptr) {
                throw std::runtime_error("Invalid task plan: multiple DELIVERY tasks are not supported");
            }
            delivery = &task;
        }
    }
    if (survey == nullptr || delivery == nullptr) {
        throw std::runtime_error("Invalid task plan: both SURVEY and DELIVERY are required");
    }
    if (!survey->predecessor_task_id.empty()) {
        throw std::runtime_error("Invalid task plan: SURVEY must not have a predecessor");
    }
    if (delivery->predecessor_task_id != survey->task_id) {
        throw std::runtime_error("Invalid task plan: DELIVERY predecessor must be the SURVEY task_id");
    }
    if (survey->incident_id != delivery->incident_id) {
        throw std::runtime_error("Invalid task plan: both tasks must share the same incident_id");
    }
    if (survey->frame_id != delivery->frame_id || survey->map_version != delivery->map_version) {
        throw std::runtime_error("Invalid task plan: both tasks must share frame_id and map_version");
    }
}

} // namespace

Plan parse_json(const std::string& json) {
    const JsonValue root = JsonParser(json).parse();
    require_exact_properties(root, {"tasks"}, "root");
    const JsonValue& tasks = require_type(require_property(root, "tasks", "root"), JsonType::Array, "root.tasks");
    Plan plan;
    plan.tasks.reserve(tasks.array.size());
    for (std::size_t index = 0; index < tasks.array.size(); ++index) {
        plan.tasks.push_back(parse_task(tasks.array[index], index));
    }
    validate_plan(plan);
    return plan;
}

Plan load_file(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Unable to open task plan: " + path.string());
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    if (!input.good() && !input.eof()) {
        throw std::runtime_error("Unable to read task plan: " + path.string());
    }
    return parse_json(contents.str());
}

} // namespace taskplan
