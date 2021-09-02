/*
 * Filter.h
 *
 *  Created on: 13.08.2021
 *      Author: Marcin
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <vector>

//EMA filter
// filteredValue - variable to be filtered
// newValue - new input value; its impact is proportional to (1-strength)
// strength - filter strength: 0.0f no filtering, 1.0f for no input value impact
template<typename Type> void filterEMA(Type& filteredValue, Type newValue, float strength)
{
    filteredValue = strength * filteredValue + (1.0F - strength) * newValue;
}

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

class MovingAverageFilter
{
public:
    explicit MovingAverageFilter(size_t size);
    float getFilterValue() const { return sum / size; }
    void filter(float inputData);
private:
    size_t size;
    std::vector<float> data;      //input data vector
    float sum{0.0F};      //current sum of all elements
};

#endif /* FILTER_H_ */