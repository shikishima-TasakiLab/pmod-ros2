#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <yaml.h>

namespace yaml {

enum ValueType
{
    InvalidType,
    TypeBool,
    TypeInt,
    TypeFloat,
    TypeString,
    TypeArray,
    TypeStruct
};

class Any;

static const char* TYPEID_BOOL = typeid(bool).name();
static const char* TYPEID_INT = typeid(long).name();
static const char* TYPEID_FLOAT = typeid(double).name();
static const char* TYPEID_STRING = typeid(std::string).name();
static const char* TYPEID_ARRAY = typeid(std::vector<Any>).name();
static const char* TYPEID_STRUCT = typeid(std::map<std::string, Any>).name();

class Any
{
public:
    Any(const bool &&value): _value(new Any_Derived<bool>(value)) {}
    Any(const int &value): _value(new Any_Derived<long>(value)) {}
    Any(const long &&value): _value(new Any_Derived<long>(value)) {}
    Any(const float &&value): _value(new Any_Derived<double>(value)) {}
    Any(const double &&value): _value(new Any_Derived<double>(value)) {}
    Any(const std::string &value): _value(new Any_Derived<std::string>(value)) {}
    Any(const std::vector<Any> &value): _value(new Any_Derived<std::vector<Any>>(value)) {}
    Any(const std::map<std::string, Any> &value): _value(new Any_Derived<std::map<std::string, Any>>(value)) {}
    ~Any(){delete _value;}

    enum ValueType get_type() {return this->_value->type;}

private:
    class Any_Base
    {
    public:
        virtual ~Any_Base(){}
        enum ValueType type;
    };
    template<class T>
    class Any_Derived : public Any_Base
    {
    public:
        Any_Derived(T const &value): _value(value) {
            const char* type_of_value = typeid(value).name();

            if (!strcmp(type_of_value, TYPEID_BOOL)) this->type = ValueType::TypeBool;
            else if (!strcmp(type_of_value, TYPEID_INT)) this->type = ValueType::TypeInt;
            else if (!strcmp(type_of_value, TYPEID_FLOAT)) this->type = ValueType::TypeFloat;
            else if (!strcmp(type_of_value, TYPEID_STRING)) this->type = ValueType::TypeString;
            else if (!strcmp(type_of_value, TYPEID_ARRAY)) this->type = ValueType::TypeArray;
            else if (!strcmp(type_of_value, TYPEID_STRUCT)) this->type = ValueType::TypeStruct;
            else this->type = ValueType::InvalidType;
        }
    private:
        T _value;
    };
    Any_Base *_value;
};

void indent(int level);
void print_event(yaml_event_t *event);
Any parse_mapping(yaml_parser_t *parser, yaml_event_t *event);

Any parse_yaml(const std::string &path);

}   //  yaml
