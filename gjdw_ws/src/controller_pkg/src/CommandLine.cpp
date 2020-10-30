#include "CommandLine.h"

#include <iostream>

using namespace std;

std::ostream& operator<<(std::ostream& os, const CommandLine& cmd_line)
{
    os << "(" << cmd_line.getLineNo() << ") " << cmd_line.getContent();
    return os;
}