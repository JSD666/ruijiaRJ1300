#ifndef VARIABLE_H
#define VARIABLE_H

#include <string>
#include <memory>
#include <algorithm>
#include <iosfwd>
#include <map>

#include "Value.h"

// 定义作业文件使用的变量类型
struct Variable
{
public:
    enum Type
    {
        VARIABLE_TYPE_UNKNOWN,
        VARIABLE_TYPE_NONE,
        VARIABLE_TYPE_INT,
        VARIABLE_TYPE_DOUBLE,
        VARIABLE_TYPE_DOUBLE_ARRAY,
        VARIABLE_TYPE_DOUBLE_6,
        VARIABLE_TYPE_STRING,
        VARIABLE_TYPE_UINT32,
    };

    Variable() : type_(VARIABLE_TYPE_NONE), value_pointer_(nullptr) { }

    Variable(const Variable& rhs);

    Variable(const std::string& name, const std::string& type_name, const Variable::Type& type)
        : name_(name), type_name_(type_name), type_(type), value_pointer_(nullptr)
    {
        createValue(type_);
    }

    // 交换两个变量的内容
    void swap(Variable& rhs)
    {
        std::swap(name_, rhs.name_);
        std::swap(type_name_, rhs.type_name_);
        std::swap(type_, rhs.type_);
        std::swap(value_pointer_, rhs.value_pointer_);
    }

    // Copy-swap实现自赋值安全和异常安全
    Variable& operator=(const Variable& rhs)
    {
        Variable temp(rhs);
        swap(temp);
        return *this;
    }

    ~Variable();

    // 提取值
    template<typename T>
    const T& getValue() const
    {
        T* value_pointer = static_cast<T*>(value_pointer_);
        return *value_pointer;
    }

    std::string getName() const { return name_; }

    std::string getTypeName() const { return type_name_; }

    Variable::Type getType() const { return type_; }

    // 设置值
    template<typename T>
    void setValue(const T& t)
    {
        if (!value_pointer_) // 允许直接将值赋给空变量
        {
            createValue(t);

            if (typeid(T) == typeid(IntValue))
                type_ = VARIABLE_TYPE_INT;
            else if (typeid(T) == typeid(DoubleValue))
                type_ = VARIABLE_TYPE_DOUBLE;
            else if (typeid(T) == typeid(DoubleValueArray))
                type_ = VARIABLE_TYPE_DOUBLE_ARRAY;
        }
        else
        {
            T* p = static_cast<T*>(value_pointer_);
            *p = t;
        }
    }

    void setName(const std::string& name) { name_ = name; }

    void setType(const Variable::Type& type) { type_ = type; }

    void setTypename(const Variable::Type& type_name) { type_name_ = type_name; }

    friend std::ostream& operator<<(std::ostream& os, const Variable& v);
private:
    // 分配内存给变量值
    void createValue(const Variable::Type& type);

    template<typename T>
    void createValue(const Variable::Type& type, const T& t)
    {
        createValue(t);
        if (value_pointer_) type_ = type;
    }

    template<typename T>
    void createValue(const T& t);

    // 释放变量值使用的内存
    void releaseValue();

    std::string name_;      // 变量名
    std::string type_name_;  // 变量类型名
    Type type_;             // 变量类型
    void* value_pointer_;   // 用void*抽象值类型
};

using VariablePointer = std::shared_ptr<Variable>;
using VariableTable = std::map<std::string, VariablePointer>;
using VariableTablePointer = VariableTable*;

std::ostream& operator<<(std::ostream& os, const VariableTable& variable_table);
std::ostream& operator<<(std::ostream& os, const DoubleValueArray& vec);

#endif // VARIABLE_H