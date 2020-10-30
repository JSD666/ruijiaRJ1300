#include "Segment.h"

#include <iostream>
#include <algorithm>
#include <iterator>

using namespace std;

// 格式化输出Segment
std::ostream& operator<<(std::ostream& out, const Segment& segment)
{
    out << "[Segment " << segment.no_ << " " << segment.name_ << "]\n"
        << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"
        << segment.blocks_;
    if (!segment.parallel_blocks_.empty())
    {
        out << "==========Parallel===========\n"
            << segment.parallel_blocks_;
    }
    out << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
    return out;
}

std::ostream& operator<<(std::ostream& out, const Segments& segments)
{
    copy(segments.cbegin(), segments.cend(), ostream_iterator<Segment>(out, "\n"));
    return out;
}