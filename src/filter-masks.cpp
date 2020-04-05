#include "filter-masks.hpp"

#include <exception>

using namespace std;

template<class T>
vector<T> applyIndexMask(const vector<T>& input, const vector<unsigned int>& mask) {
    vector<T> output;

    for (auto index : mask)
        output.push_back(output.at(index));

    return output;
}


template<class T>
vector<T> applyMask(const vector<T>& input, const vector<unsigned char>& mask) {
    vector<T> output;

    if (!mask.size() == input.size())
        throw runtime_error("Input und mask do not have the same length.");

    for(size_t i = 0; i < mask.size(); i++) {
        if (mask[i])
            output.push_back(input[i]);
    }

    return output;
}
