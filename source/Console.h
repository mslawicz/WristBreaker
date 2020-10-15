/*
 * Console.h
 *
 *  Created on: 22.12.2019
 *      Author: Marcin
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include "mbed.h"
#include <string>
#include <vector>
#include <map>
#include <utility>

enum class KeyCode : int
{
    LF = 10,
    CR = 13,
    Escape = 27,
    Tilde = 126,
    Backspace = 8,
    Delete = 127,
    Space = 32
};

using CommandVector = std::vector<std::string>;
using CommandContainer = std::pair<std::string, Callback<void(const CommandVector&)>>;

class Console
{
public:
    static Console& getInstance();  // NOLINT(modernize-use-trailing-return-type)
    Console(Console const&) = delete;   // copy constructor removed for singleton
    void operator=(Console const&) = delete;
    Console(Console&&) = delete;
    void operator=(Console&&) = delete;
    void handler();
    void registerCommand(const std::string& command, const std::string& helpText, Callback<void(const CommandVector&)> commandCallback);
    void displayHelp(const CommandVector&);
private:
    Console() = default; // private constructor definition
    ~Console() = default;
    void executeCommand();
    CommandVector commandElements;
    std::map<std::string, CommandContainer> commands;
};

#endif /* CONSOLE_H_ */
