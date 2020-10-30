#ifndef SEGMENT_H
#define SEGMENT_H

#include <string>
#include <vector>
#include <iosfwd>

#include "CommandBlock.h"

struct Segment
{
    std::string name_; // 段名；有中文乱码问题
    int no_; // 段号
    std::string content_; // 段内容
    CommandBlocks blocks_; // 段块
    CommandBlocks parallel_blocks_; // 并行段块

    Segment() : no_(-1) { }
 
    // Reset Segment
    void clear() { name_.clear(); no_ = -1; content_.clear(); blocks_.clear(); parallel_blocks_.clear(); }

    // 格式化输出Segment
    friend std::ostream& operator<<(std::ostream& out, const Segment& segment);        
};

// 别名定义
using Segments = std::vector<Segment>;

std::ostream& operator<<(std::ostream& out, const Segments& segments); 

#endif // SEGMENT_H