/*
 * Filter.h
 *
 *  Created on: 13.08.2021
 *      Author: Marcin
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <vector>
 
class MedianFilter
{
public:
    explicit MedianFilter(size_t size);
    float getMedian(float newValue);    //filter and return current value
private:
    size_t size;
    std::vector<float> chronoData;      //data ordered by time
    std::vector<float> sortedData;      //data ordered by value
};

#endif /* FILTER_H_ */