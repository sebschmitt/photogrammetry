#ifndef YAPGT_ARGPARSER_H
#define YAPGT_ARGPARSER_H

#include <string>
#include <sstream>
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

        // this works due to c++s operator overloading for the >> operator
        template<typename T> T getValue() {
            std::istringstream in(this->value);
            T t = T();
            in >> t >> std::ws;
            return t;
        }

        Argument(std::string name, std::string description);

    protected:
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
