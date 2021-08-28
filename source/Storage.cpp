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
    char key[MaxKeySize] = {0};
    std::cout << "Stored parameters: ";
    while(kv_iterator_next(it, static_cast<char*>(key), MaxKeySize) != MBED_ERROR_ITEM_NOT_FOUND)
    {
        std::cout << key << ", ";
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