/* mbed Microcontroller Library
 * Copyright (c) 2006-2014 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Console.h"
#include "Commander.h"
#include "Logger.h"
#include "Statistics.h"
#include "Storage.h"
#include <chrono>
#include <iostream>
#include <mbed.h>

int main() // NOLINT(modernize-use-trailing-return-type)
{
#ifdef MBED_DEBUG
    HAL_DBGMCU_EnableDBGSleepMode();
#endif
    logTimer.start();        //start timer for log purposes
    LOG_ALWAYS("WristBreaker v1");

    // create and start console thread
    Thread consoleThread(osPriority_t::osPriorityLow4, OS_STACK_SIZE, nullptr, "console");
    consoleThread.start(callback(&Console::getInstance(), &Console::handler));

    // register some console commands
    Console::getInstance().registerCommand("h", "help (display command list)", callback(&Console::getInstance(), &Console::displayHelp));
    Console::getInstance().registerCommand("lt", "list threads", callback(listThreads));

    // create main object
    Commander commander;

    //main loop
    const auto MainLoopSleepTime = std::chrono::milliseconds(1);
    while(true)
    {
        commander.handler();
        ThisThread::sleep_for(MainLoopSleepTime);
    }

    return 0;
}

