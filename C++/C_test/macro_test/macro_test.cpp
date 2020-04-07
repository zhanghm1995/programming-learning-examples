#include <iostream>

#include "class_define.h"
#include "macro_define.h"

using namespace std;

/// Define some useful printf macro
#define AINFO(format, ...) printf("[INFO] " format "\n", ##__VA_ARGS__)
#define AWARN(format, ...) printf("[WARN]" format "\n", ##__VA_ARGS__)

int main()
{
    Example1 example1;
    cout<<example1.GetLength()<<endl;
    cout<<MaxLength<<endl;

    cout<<"================="<<endl;
    AINFO("This is number %d", 10);
    AWARN("Failed to load %s", "arguments");
    return 0;
}
