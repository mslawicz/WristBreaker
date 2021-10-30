/*
 * Arbiter.h
 *
 *  Created on: 29.10.2021
 *      Author: Marcin
 */

#ifndef ARBITER_H_
#define ARBITER_H_

#include <stdint.h>
#include <utility>

template <class T>
class Arbiter
{
public:
    std::pair<bool,bool> valueChanged(T remote, T local, uint16_t counterLoad);
private:
    T lastRemote{0};
    T lastLocal{0};
    
    uint16_t remoteCounter{0};
    uint16_t localCounter{0};
};

template <class T>
std::pair<bool,bool> Arbiter<T>::valueChanged(T remote, T local, uint16_t counterLoad)
{
    if ((remote != lastRemote) && (0 == localCounter))
    {
        remoteCounter = counterLoad;
    }

    if ((local != lastLocal) && (0 == remoteCounter))
    {
        localCounter = counterLoad;
    }

    lastRemote = remote;
    lastLocal = local;

    if (0 < remoteCounter)
    {
        remoteCounter--;
    }

    if (0 < localCounter)
    {
        localCounter--;
    }

    return std::pair<bool,bool>{remoteCounter != 0, localCounter !=0};
}

#endif /*ARBITER_H_*/