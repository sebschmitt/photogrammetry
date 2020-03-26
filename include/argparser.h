//
// Created by Sebastian Schmitt on 19.03.2020.
//

#ifndef YAPGT_ARGPARSER_H
#define YAPGT_ARGPARSER_H

#include <iostream>
#include <vector>


namespace argparser {
    class Argument {
        friend class ArgumentParser;

    public:
        std::string getDescription();
        std::string getName();
        Argument* setRequired(bool);
        bool isFound();

        // I have no fucking clue why this works.
        template<typename T> T getValue() {
            std::istringstream in(_get<std::string>());
            T t = T();
            in >> t >> std::ws;
            return t;
        }

        Argument(std::string name, std::string description);

    protected:
        template<typename T> T _get() {
            T t = T();
            typename T::value_type vt;
            for (auto &s : this->value) {
                std::istringstream in(&s);
                in >> vt;
                t.push_back(vt);
            }

            return t;
        }

        std::string value;
        bool _found{false};
        bool _required{false};
        std::string name;
        std::string description;

    };

    class ArgumentParser {
    public:
        ArgumentParser(std::string);

        void addArgument(Argument *);
        void parseArguments(int argc, char *argv[]);
        void printHelp();

    private:
        std::vector<Argument *> _arguments{};
        std::string app_name;
        Argument _help = Argument("help", "Get help!");


    };
}


#endif //YAPGT_ARGPARSER_H
