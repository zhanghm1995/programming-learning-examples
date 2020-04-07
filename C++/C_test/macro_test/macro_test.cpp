#include <iostream>

#include "class_define.h"
#include "macro_define.h"

using namespace std;

/// Define some useful printf macro
#define AINFO(format, ...) printf("[INFO] " format "\n", ##__VA_ARGS__)
#define AWARN(format, ...) printf("\033[1;33m[WARN] " format "\033[0m\n", ##__VA_ARGS__)
#define AERROR(format, ...) printf("\033[1;31m[ERROR] " format "\033[0m\n", ##__VA_ARGS__)
void PrintInfoWithColor() {
    printf("\033[1;33mHello, world!\033[0m\n"); // yellow

    int count = 10;
    printf("\033[1;31m[INFO] There are %d images for processing...\033[0m\n", count);
}

int main()
{
    Example1 example1;
    cout<<example1.GetLength()<<endl;
    cout<<MaxLength<<endl;

    cout<<"================="<<endl;
    PrintInfoWithColor();

    cout<<"================="<<endl;
    AINFO("This is number %d", 10);
    AWARN("Delta time is less than %d", 1);
    AERROR("Failded to init!");
    return 0;
}
