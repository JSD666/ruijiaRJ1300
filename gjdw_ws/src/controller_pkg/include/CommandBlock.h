#ifndef COMMAND_BLOCK_H
#define COMMAND_BLOCK_H

#include <string>
#include <vector>
#include <iosfwd>

#include "CommandLine.h"

using CommandLines = std::vector<CommandLine>;

struct CommandBlock
{
    // 命令块类型定义
    enum Type
    {
        COMMAND_BLOCK_TYPE_SEQUENTIAL,    // 顺序块，逐行顺序执行
        COMMAND_BLOCK_TYPE_PARALLEL,      // 并行块，段间逐行并行
        COMMAND_BLOCK_TYPE_CALCULATION,   // 计算块，进行变量计算
    };

    CommandBlock() : type_(COMMAND_BLOCK_TYPE_SEQUENTIAL) { }
    explicit CommandBlock(const Type type) : type_(type) { }
    CommandBlock(const Type type, const CommandLines& cmd_lines)
        : type_(type), cmd_lines_(cmd_lines) { }

    // 清空块数据
    void clear() { type_ = COMMAND_BLOCK_TYPE_SEQUENTIAL; cmd_lines_.clear(); }

    // 返回块中命令行数量
    size_t size() const { return cmd_lines_.size(); }

    // 判断块中是否包含命令
    bool empty() const { return cmd_lines_.empty(); }

    // -----------------
    // ----数据成员访问接口
    // -----------------
    Type getType() const { return type_; }
    void setType(const Type type) { type_ = type; }

    const CommandLines& getCommandLines() const { return cmd_lines_; }
    CommandLines& getCommandLines() { return cmd_lines_; }

    void addCommandLine(const CommandLine& line) { cmd_lines_.push_back(line); }

    // 格式化输出
    friend std::ostream& operator<<(std::ostream& out, const CommandBlock& block);

private:
    Type type_; // 块类型
    CommandLines cmd_lines_; // 块内容
};

using CommandBlocks = std::vector<CommandBlock>; // 命令块集

std::ostream& operator<<(std::ostream& out, const CommandBlocks& blocks);
std::string to_string(const CommandBlock::Type block_type);

#endif // COMMAND_BLOCK_H