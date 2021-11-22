/*
 * Logger.h
 *
 *  Created on: 26.10.2021
 *      Author: Marcin
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <mbed.h>
#include <sstream>

 
extern Timer logTimer;      //NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
 
enum struct LogLevel : uint8_t
{
    None,
    Always,
    Error,
    Warning,
    Info,
    Debug
};
 
//////// set current log level /////////
constexpr LogLevel currentLevel = LogLevel::Info;

void logMessage(LogLevel level, std::ostream& ostr);

#define LOG_ALWAYS(message) logMessage(LogLevel::Always, std::ostringstream().flush() << message)     //NOLINT(cppcoreguidelines-macro-usage,bugprone-macro-parentheses) 
#define LOG_ERROR(message) logMessage(LogLevel::Error, std::ostringstream().flush() << message)     //NOLINT(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)
#define LOG_WARNING(message) logMessage(LogLevel::Warning, std::ostringstream().flush() << message) //NOLINT(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)
#define LOG_INFO(message) logMessage(LogLevel::Info, std::ostringstream().flush() << message)       //NOLINT(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)
#define LOG_DEBUG(message) logMessage(LogLevel::Debug, std::ostringstream().flush() << message)      //NOLINT(cppcoreguidelines-macro-usage,bugprone-macro-parentheses)
 
#endif /* LOGGER_H_ */