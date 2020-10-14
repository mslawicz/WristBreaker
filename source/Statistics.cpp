/*
 * Statistics.cpp
 *
 *  Created on: 22.12.2019
 *      Author: Marcin
 */

#include <mbed.h>
#include "mbed_stats.h"
#include "Statistics.h"

#if !defined(MBED_THREAD_STATS_ENABLED)
#error "Stats not enabled"
#endif

void listThreads(CommandVector cv)
{
    mbed_stats_thread_t* stats = new mbed_stats_thread_t[MAX_THREAD_STATS];
    int numberOfThreads = mbed_stats_thread_get_each(stats, MAX_THREAD_STATS);

    printf("ID,Name,State,Priority,Stack size, Stack space\r\n");
    for(int i = 0; i < numberOfThreads; i++)
    {
        printf("0x%X,%s,%u,%u,%u,%u\r\n",
                (unsigned int)stats[i].id,
                stats[i].name,
                (unsigned int)stats[i].state,
                (unsigned int)stats[i].priority,
                (unsigned int)stats[i].stack_size,
                (unsigned int)stats[i].stack_space);
    }
}