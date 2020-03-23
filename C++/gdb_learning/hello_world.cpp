#include <iostream>
#include <string>
#include <memory>

int main()
{
    std::cout<<"hello world"<<std::endl;
    std::shared_ptr<std::string> Ptr;
    std::cout<<"Ptr is "<<Ptr->c_str()<<std::endl;
    return 0;
}