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
// newValue - new input value; its impact is proportional to alpha constant
// alpha - smoothing factor: 0.0f for no input value impact; 1.0f no filtering 
template<typename Type> void filterEMA(Type& filteredValue, Type newValue, float alpha)
{
    filteredValue = (1.0F - alpha) * filteredValue + alpha * newValue;
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
    float getFilterValue(float inputData);       //filter and return current value
private:
    size_t size;
    std::vector<float> data;      //input data vector
    float sum{0.0F};      //current sum of all elements
};

//adaptive exponential moving average filter
class AEMAFilter
{
public:
    float calculate(float input, float alpha);
private:
    float filteredValue{0.0F};              // current filtered value
    const float DeviationAlpha = 0.02F;     // filter strength for average deviation calculations
    float filteredDeviation{0.0F};          // current filtered deviation value 
};

#endif /* FILTER_H_ */