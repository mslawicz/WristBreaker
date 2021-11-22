#include "Logger.h"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <ratio>
#include <vector>


Timer logTimer;     //NOLINT(fuchsia-statically-constructed-objects,cppcoreguidelines-avoid-non-const-global-variables)

void logMessage(LogLevel level, std::ostream& ostr)     //NOLINT(misc-definitions-in-headers)
{  
    const std::vector<std::string>levelText{"None", ":", "Error", "Warning", "Info", "Debug"};
    auto levelValue = static_cast<uint8_t>(level);
    if(level <= currentLevel)
    {
        constexpr uint32_t UsInSec = 1000000U;
        constexpr uint8_t UsDigits = 6U;
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(logTimer.elapsed_time()).count();
        std::cout << "\r[" << us / UsInSec << "." << std::setfill('0') << std::setw(UsDigits) << us % UsInSec << "] ";
        std::cout << levelText[levelValue] << ": ";
        std::cout << ((std::ostringstream&)ostr).str() << std::endl;    //NOLINT(cppcoreguidelines-pro-type-cstyle-cast,google-readability-casting)
        std::cout << ">" << std::flush;
    }
}