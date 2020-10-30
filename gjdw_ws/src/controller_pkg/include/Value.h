#ifndef VALUE_H
#define VALUE_H

#include <string>
#include <vector>
#include <iosfwd>

// 定义作业文件使用的变量值类型
template<typename T>
class Value
{
public:
    Value() { }
    Value(const T& v) : v_(v) { }

    T get() const { return v_; } 
    T& get() { return v_; } 
    void set(const T& v) { v_ = v; }

    operator T() const
    {
        return v_;
    }
    // friend std::ostream& operator<<(std::ostream& os, const Value& v);
private:
    T v_;
};

// using IntValue = Value<int>;
// using DoubleValue = Value<double>;
// using DoubleValueArray = std::vector<DoubleValue>;
// using StringValue = Value<std::string>;

using IntValue = int;
using DoubleValue = double;
using DoubleValueArray = std::vector<DoubleValue>;
using StringValue = std::string;

#endif // VALUE_H