#ifndef STORAGE_H_
#define STORAGE_H_

#include "Console.h"
#include "KVStore.h"
#include "kvstore_global_api.h"
#include "mbed.h"
#include <iostream>
#include <string>

/*
class of flash disk storage for program parameters
*/
class KvStore
{
public:
    KvStore(KvStore const&) = delete;       // do not allow copy constructor of a singleton
    void operator=(KvStore const&) = delete;
    KvStore(KvStore&&) = delete;
    void operator=(KvStore&&) = delete;
    static KvStore& getInstance();
    size_t restoreData(std::string& key, void* pData);
    
    template<typename T> void store(const std::string key, T value)     // store key-value pair in memory
    {
        int result = kv_set(key.c_str(), &value, sizeof(T), 0);
        if(result)
        {
            std::cout << "Parameter '" << key << "' store error " << MBED_GET_ERROR_CODE(result) << std::endl;
        }
    }

    /*
    restore value from the given key using min-max limits
    if key not found, create the parameter with default value
    */
    // template<typename T> T restore(const std::string key, T defaultValue, T minValue, T maxValue)
    // {
    //     T value = restore<T>(key, defaultValue);
    //     if(value > maxValue)
    //     {
    //         value = maxValue;
    //     }
    //     else if(value < minValue)
    //     {
    //         value = minValue;
    //     }

    //     return value;
    // }

    static void list(const CommandVector& cv);
    static void clear(const CommandVector& cv);
private:
    KvStore();
    ~KvStore();
    kv_info_t info{0, 0};
};

#endif /* STORAGE_H_ */
