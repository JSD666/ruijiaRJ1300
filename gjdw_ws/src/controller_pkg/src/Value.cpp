#include "Value.h"

#include <iostream>
#include <iterator>

using namespace std;

template<typename T>
std::ostream& operator<<(std::ostream& os, const Value<T>& v)
{
    return os;
}

std::ostream& operator<<(std::ostream& os, const DoubleValueArray& vec)
{
    copy(vec.cbegin(), vec.cend(), ostream_iterator<double>(os, " "));
    return os;
}