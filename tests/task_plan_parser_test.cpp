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

        std::cout << "C++ task-plan parser tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "C++ task-plan parser test failed: " << error.what() << "\n";
        return 1;
    }
}
