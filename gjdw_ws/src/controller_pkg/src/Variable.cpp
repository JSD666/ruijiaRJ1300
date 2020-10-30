#include "Variable.h"

#include <iostream>
#include <map>
#include <algorithm>
#include <iterator>

using namespace std;

Variable::Variable(const Variable& rhs)
    : name_(rhs.name_), type_name_(rhs.type_name_), type_(rhs.type_), value_pointer_(nullptr)
{
    if (!rhs.value_pointer_)
    {
        name_.clear();
        type_name_.clear();
        type_ = VARIABLE_TYPE_NONE;
        return;
    }

    // 根据类型分配值
    switch (rhs.type_)
    {
    case VARIABLE_TYPE_INT:
        createValue(rhs.type_, rhs.getValue<IntValue>());
        break;

    case VARIABLE_TYPE_DOUBLE:
        createValue(rhs.type_, rhs.getValue<DoubleValue>());
        break;

    case VARIABLE_TYPE_DOUBLE_ARRAY:
    case VARIABLE_TYPE_DOUBLE_6:
        createValue(rhs.type_, rhs.getValue<DoubleValueArray>());
        break;

    default:
        break;
    }
}

void Variable::createValue(const Variable::Type& type)
{
    if (value_pointer_) releaseValue();

    // 根据类型分配值
    switch (type)
    {
    case VARIABLE_TYPE_INT:
        createValue(type, IntValue(0));
        break;

    case VARIABLE_TYPE_DOUBLE:
        createValue(type, DoubleValue(0));
        break;

    case VARIABLE_TYPE_DOUBLE_ARRAY:
        createValue(type, DoubleValueArray());
        break;

    case VARIABLE_TYPE_DOUBLE_6:
        createValue(type, DoubleValueArray(6, 0));
        break;

    case VARIABLE_TYPE_NONE:
    default:
        value_pointer_ = nullptr;
        break;
    }
    type_ = type;
}

template<typename T>
void Variable::createValue(const T& t)
{
    if (value_pointer_) releaseValue();

    try {
        value_pointer_ = new T(t);
    } catch (const std::exception& e) {
        cerr << "Failed to new Value: " << e.what() << endl; 

        name_.clear();
        value_pointer_ = nullptr;
        type_ = VARIABLE_TYPE_NONE;
    }
}

void Variable::releaseValue()
{
    if (!value_pointer_) return;

    void* value_pointer = value_pointer_;
    const Type type = type_;

    value_pointer_ = nullptr;
    type_ = VARIABLE_TYPE_NONE;

    switch (type)
    {
    case VARIABLE_TYPE_INT:
        delete static_cast<IntValue*>(value_pointer);
        break;

    case VARIABLE_TYPE_DOUBLE:
        delete static_cast<DoubleValue*>(value_pointer);
        break;

    case VARIABLE_TYPE_DOUBLE_ARRAY:
    case VARIABLE_TYPE_DOUBLE_6:
        delete static_cast<DoubleValueArray*>(value_pointer);
        break;

    case VARIABLE_TYPE_NONE:
    default:
        break;
    }
}

Variable::~Variable()
{
    releaseValue();
}

std::ostream& operator<<(std::ostream& os, const Variable& v)
{
    os << "[Variable " << v.getName() << "@" << v.getTypeName() << "]: ";
    switch (v.getType())
    {
    case Variable::VARIABLE_TYPE_INT:
        os << v.getValue<IntValue>();
        break;
    
    case Variable::VARIABLE_TYPE_DOUBLE:
        os << v.getValue<DoubleValue>();
        break;
    
    case Variable::VARIABLE_TYPE_DOUBLE_ARRAY:
    case Variable::VARIABLE_TYPE_DOUBLE_6:
        {
            const DoubleValueArray& values = v.getValue<DoubleValueArray>();
            copy(values.cbegin(), values.cend(), ostream_iterator<double>(os, " "));
        }
        break;
    
    default:
        break;
    }

    return os;
}

std::ostream& operator<<(std::ostream& os, const VariableTable& variable_table)
{
    for (const auto& p : variable_table)
    {
        os << *p.second << endl;
    }
    return os;
}