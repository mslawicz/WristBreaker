#include "Storage.h"

KvStore::KvStore()
{
    Console::getInstance().registerCommand("lp", "list stored parameters", callback(this, &KvStore::list));
    Console::getInstance().registerCommand("cp", "clear all stored parameters", callback(this, &KvStore::clear));
}

KvStore& KvStore::getInstance()
{
    static KvStore instance;  // Guaranteed to be destroyed, instantiated on first use
    return instance;
}

/*
list all stored parameter keys
*/
void KvStore::list(CommandVector cv)
{
    kv_iterator_t it;
    int result = kv_iterator_open(&it, NULL);
    if(result)
    {
        printf("Error %d on parameters iteration\n", MBED_GET_ERROR_CODE(result));
        return;
    }
    const size_t MaxKeySize = 50;
    char key[MaxKeySize] = {0};
    printf("Stored parameters: ");
    while(kv_iterator_next(it, key, MaxKeySize) != MBED_ERROR_ITEM_NOT_FOUND)
    {
        printf("%s, ", key);
        memset(key, 0, MaxKeySize);
    }
    kv_iterator_close(it);
    printf("\n");
}

/*
clear storage memory
*/
void KvStore::clear(CommandVector cv)
{
    int result = kv_reset("/kv/");
    if(result)
    {
        printf("Resetting parameter storage failed with error %d\n", MBED_GET_ERROR_CODE(result));
    }
    else
    {
        printf("Parameter storage cleared\n");
    }
}