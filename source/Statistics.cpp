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
#include <vector>

#if !defined(MBED_THREAD_STATS_ENABLED)
#error "Stats not enabled"
#endif

void listThreads(const CommandVector&  /*cv*/)
{
    auto* statsArray = new mbed_stats_thread_t[MAX_THREAD_STATS];    // NOLINT(cppcoreguidelines-owning-memory)
    size_t numberOfThreads = mbed_stats_thread_get_each(statsArray, MAX_THREAD_STATS);
    std::vector<mbed_stats_thread_t> threads(statsArray, statsArray + numberOfThreads);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    std::cout << "ID,Name,State,Priority,Stack size, Stack space\r\n";
    for(auto thread : threads)
    {
        std::cout << std::hex << "0x" << thread.id << ","
        << std::dec << thread.name << ","
        << thread.state << ","
        << thread.priority << ","
        << thread.stack_size << ","
        << thread.stack_space << std::endl;
    }
}