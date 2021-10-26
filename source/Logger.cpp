#include "Logger.h"
#include <chrono>
#include <vector>

Timer logTimer;     //NOLINT(fuchsia-statically-constructed-objects,cppcoreguidelines-avoid-non-const-global-variables)

void logMessage(LogLevel level, std::ostream& ostr)     //NOLINT(misc-definitions-in-headers)
{  
    const std::vector<std::string>levelText{"None", "Error", "Warning", "Info", "Debug"};
    auto levelValue = static_cast<uint8_t>(level);
    if(level <= currentLevel)
    {
        std::cout << "\r[" << std::chrono::duration<float, std::milli>(logTimer.elapsed_time()).count() << "] ";
        std::cout << levelText[levelValue] << ": ";
        std::cout << ((std::ostringstream&)ostr).str() << std::endl;    //NOLINT(cppcoreguidelines-pro-type-cstyle-cast,google-readability-casting)
        std::cout << ">" << std::flush;
    }
}