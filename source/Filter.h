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
    float getMedian(float newValue);
private:
    size_t size;
    std::vector<float> chronoData;
    std::vector<float> sortedData;
};

#endif /* FILTER_H_ */