#include "Storage.h"

KvStore::KvStore()
{
    Console::getInstance().registerCommand("lsp", "list stored parameters", callback(&KvStore::list));
    Console::getInstance().registerCommand("csp", "clear all stored parameters", callback(&KvStore::clear));    
}

KvStore& KvStore::getInstance()
{
    static KvStore instance;  // Guaranteed to be destroyed, instantiated on first use
    return instance;
}

/*
list all stored parameter keys
*/
void KvStore::list(const CommandVector&  /*cv*/)
{
    kv_iterator_t it = nullptr;
    int result = kv_iterator_open(&it, nullptr);
    if(result != 0)
    {
        std::cout << "Error " << MBED_GET_ERROR_CODE(result) << " on parameters iteration" << std::endl;
        return;
    }
    const size_t MaxKeySize = 50;
    char key[MaxKeySize] = {0}; //NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
    std::cout << "Stored parameters: ";
    while(kv_iterator_next(it, static_cast<char*>(key), MaxKeySize) != MBED_ERROR_ITEM_NOT_FOUND)
    {
        std::cout << static_cast<char*>(key) << ", ";
        memset(static_cast<char*>(key), 0, MaxKeySize);
    }
    kv_iterator_close(it);
    std::cout << std::endl;
}

/*
clear storage memory
*/
void KvStore::clear(const CommandVector&  /*cv*/)
{
    int result = kv_reset("/kv/");
    if(result != 0)
    {
        std::cout << "Resetting parameter storage failed with error " << MBED_GET_ERROR_CODE(result) << std::endl;
    }
    else
    {
        std::cout << "Parameter storage cleared" << std::endl;
    }
}

/*
restore data from the given key
returns the actual size od restored data or 0 if error
*/
size_t KvStore::restoreData(std::string& key, void* pData)
{
    int error = kv_get_info(key.c_str(), &info);
    if(0 != error)
    {
        std::cout << "Parameter '" << key << "' info error " << MBED_GET_ERROR_CODE(error) << std::endl;
        return 0;
    }
            
    size_t actualSize{0};
    error = kv_get(key.c_str(), pData, info.size, &actualSize);
    if(0 != error)
    {
        std::cout << "Parameter '" << key << "' restore error " << MBED_GET_ERROR_CODE(error) << std::endl;
        return 0;
    }
    
    return actualSize;
}

/*
store data with the given key
returns the error code or 0 if OK
*/
int KvStore::storeData(std::string& key, const void* pData, size_t size)
{
    int error = kv_set(key.c_str(), pData, size, 0);
    if(error)
    {
        std::cout << "Parameter '" << key << "' store error " << MBED_GET_ERROR_CODE(error) << std::endl;
    }
    return error;
}
