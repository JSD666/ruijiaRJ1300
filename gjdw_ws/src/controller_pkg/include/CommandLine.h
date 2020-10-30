#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <string>
#include <cstddef>
#include <iosfwd>

// 命令行抽象数据类型
class CommandLine
{
public:
    CommandLine() : line_no_(0) { }
    explicit CommandLine(const std::string& content, const size_t line_no = 0)
        : content_(content), line_no_(line_no) { }

    const std::string& getContent() const { return content_; }
    std::string& getContent() { return content_; }

    const size_t& getLineNo() const { return line_no_; }
    size_t& getLineNo() { return line_no_; }

    void setContent(const std::string& content) { content_ = content; }
    void setLineNo(const size_t line_no) { line_no_ = line_no; }

    inline bool operator<(const CommandLine& rhs) const { return line_no_ < rhs.line_no_; }
private:
    std::string content_;   // 行内容
    size_t line_no_;         // 行号
};

std::ostream& operator<<(std::ostream& os, const CommandLine& cmd_line);

#endif // COMMAND_LINE_H