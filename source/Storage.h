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
    int storeData(std::string& key, const void* pData, size_t size);
    static void list(const CommandVector& cv);
    static void clear(const CommandVector& cv);
private:
    KvStore();
    ~KvStore();
    kv_info_t info{0, 0};
};

#endif /* STORAGE_H_ */
