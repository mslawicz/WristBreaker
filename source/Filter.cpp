#include "Filter.h"

MedianFilter::MedianFilter(size_t size) : 
    size(size)
{
    chronoData.assign(size, 0.0F);
    sortedData.assign(size, 0.0F);
}
 
float MedianFilter::getMedian(float newValue)
{
    //remember and remove the oldest element
    float oldestData = chronoData.back();
    chronoData.pop_back();
    //insert the new element in the front
    chronoData.insert(chronoData.begin(), newValue);
    //find and remove the oldest element from the sorted vector
    auto it = sortedData.begin();
    while(it < sortedData.end())
    {
        if(*it == oldestData)
        {
            sortedData.erase(it);
            break;
        }
        it++;
    }
    //find a place in the sorted vector
    it = sortedData.begin();
    while(it < sortedData.end())
    {
        if(newValue < *it)
        {
            break;
        }
        it++;
    }
    //insert the new element in the sorted vector
    sortedData.insert(it, newValue);
    //return the median value
    return sortedData[size / 2];
}

MovingAverageFilter::MovingAverageFilter(size_t size) :
    size(size)
{
    data.resize(size, 0.0F);
}
 
float MovingAverageFilter::getFilterValue(float inputData)
{
    sum -= data.front();
    data.erase(data.begin());
    sum += inputData;
    data.push_back(inputData);
    return sum / size;
}


