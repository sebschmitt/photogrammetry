//
// Created by Sebastian Schmitt on 19.03.2020.
//

#include "argparser.h"

#include <utility>
#include <iomanip>

namespace argparser {
    std::string Argument::getDescription() {
        return this->description;
    }

    std::string Argument::getName() {
        return this->name;
    }

    bool Argument::isFound() {
        return this->_found;
    }

    Argument *Argument::setRequired(bool required) {
        this->_required = required;
        return this;

    }

    Argument::Argument(std::string name, std::string description) {
        this->name = std::move(name);
        this->description = std::move(description);
    }

    ArgumentParser::ArgumentParser(std::string app_name) {
        this->app_name = std::move(app_name);
        this->_arguments.push_back(&this->_help);
    }

    void ArgumentParser::printHelp() {
        std::cout << "--- " << app_name << " ---" << std::endl;
        std::cout << "Allowed arguments: " << std::endl;

        int longest_name = 0;

        for (auto &_argument : _arguments) {
            if (_argument->name.size() > longest_name)
                longest_name = _argument->name.size();
        }
        longest_name++;

        for (auto &_argument : _arguments) {
            std::cout << std::right << std::setw(longest_name)
                      << (_argument->_required ? "*" + _argument->name : _argument->name)
                      << ": "
                      << _argument->description
                      << std::endl;
        }
        std::cout << "Arguments with a * are marked as required" << std::endl;
    }

    void ArgumentParser::addArgument(Argument *argument) {
        _arguments.push_back(argument);
    }

    void ArgumentParser::parseArguments(int argc, char *argv[]) {
        if (argc == 1) return;


        std::string all_args;
        for (int i = 1; i < argc; i++) {
            all_args += std::string(argv[i]);
            all_args += " ";
        }

        std::vector<std::string> all_names;
        std::vector<std::string> all_values;

        std::string last_name;
        std::string last_value;
        bool is_name{true};

        int i = 0;
        while (i < all_args.length()) {
            if (all_args[i] == '-') { // argument start
                if (!last_name.empty()) {
                    all_names.push_back(last_name);
                    all_values.push_back(last_value);
                }
                last_name = "";
                last_value = "";
                is_name = true;
            } else if (all_args[i] == ' ') {
                if (is_name) is_name = false;
            } else if (is_name) { // here comes the argument name
                last_name += all_args[i];
            } else { // here comes the value
                last_value += all_args[i];

            }
            i++;
        }
        // this is the last argument
        all_names.push_back(last_name);
        all_values.push_back(last_value);

        for (auto &_argument : _arguments) {
            i = 0;
            for (const auto &all_name : all_names) {
                if (_argument->getName() == all_name) {
                    _argument->_found = true;
                    _argument->value = all_values[i];
                }
                i++;
            }
        }

        for (auto &_argument : _arguments) {
            if (_argument->_required && !_argument->_found) {
                std::cout << "Argument \"" + _argument->name + "\" is required" << std::endl;
            }
        }

        if (this->_help._found) this->printHelp();
    }
}