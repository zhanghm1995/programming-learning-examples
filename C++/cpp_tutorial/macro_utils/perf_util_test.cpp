#include <iostream>
#include <thread>
#include "perf_util.h"

int main()
{
    std::string indicator = "perf_util_test";
    PERF_BLOCK_START();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    PERF_BLOCK_END_WITH_INDICATOR(indicator, "test1");
    return 0;
}