/*
 * Statistics.cpp
 *
 *  Created on: 22.12.2019
 *      Author: Marcin
 */

#include <mbed.h>
#include "mbed_stats.h"
#include "Statistics.h"
#include <iostream>

#if !defined(MBED_THREAD_STATS_ENABLED)
#error "Stats not enabled"
#endif

void listThreads(const CommandVector&  /*cv*/)
{
    auto* stats = new mbed_stats_thread_t[MAX_THREAD_STATS];    // NOLINT(cppcoreguidelines-owning-memory)
    int numberOfThreads = mbed_stats_thread_get_each(stats, MAX_THREAD_STATS);

    std::cout << "ID,Name,State,Priority,Stack size, Stack space\r\n";
    for(int i = 0; i < numberOfThreads; i++)
    {
        std::cout << std::hex << "0x" << stats[i].id << ","  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        << std::dec << stats[i].name << ","   // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        << stats[i].state << ","   // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        << stats[i].priority << ","   // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        << stats[i].stack_size << ","   // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        << stats[i].stack_space << std::endl;   // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
}