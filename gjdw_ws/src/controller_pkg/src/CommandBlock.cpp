#include "CommandBlock.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <map>
#include <cassert>

using namespace std;

std::ostream& operator<<(std::ostream& out, const CommandBlock& block)
{
    out << "[Block " << to_string(block.type_) << "]\n";
    copy(block.cmd_lines_.cbegin(), block.cmd_lines_.cend(), ostream_iterator<CommandLine>(out, "\n"));
    return out;
}

std::ostream& operator<<(std::ostream& out, const CommandBlocks& blocks)
{
    copy(blocks.cbegin(), blocks.cend(), ostream_iterator<CommandBlock>(out, "\n"));
    return out;
}

std::string to_string(const CommandBlock::Type block_type)
{
    static map<CommandBlock::Type, string> type_to_string = {
        { CommandBlock::COMMAND_BLOCK_TYPE_SEQUENTIAL, "SEQUENTIAL" },
        { CommandBlock::COMMAND_BLOCK_TYPE_PARALLEL, "PARALLEL" },
        { CommandBlock::COMMAND_BLOCK_TYPE_CALCULATION, "CALCULATION" },
    };

    assert (type_to_string.find(block_type) != type_to_string.cend());

    return type_to_string.at(block_type);
}