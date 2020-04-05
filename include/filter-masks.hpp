#ifndef FILTER_MASKS_H
#define FILTER_MASKS_H

#include <vector>

template<class T>
std::vector<T> applyIndexMask(const std::vector<T>& input, const std::vector<unsigned int>& mask);


template<class T>
std::vector<T> applyMask(const std::vector<T>& input, const std::vector<unsigned char>& mask);

#endif