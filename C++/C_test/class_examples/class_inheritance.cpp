#include <iostream>

using namespace std;

class ComponentBase {
public:
    void Init() {
        cout<<"ComponentBase::Init()"<<endl;
    }
protected:
    void Clear() {
        cout<<"ComponentBase::Clear()"<<endl;
    }

private:
    virtual void Proc() = 0; 

    virtual void InitInternal() {
        cout<<"ComponentBase::InitInternal()"<<endl;
    }

    void Reset() {
         cout<<"ComponentBase::Reset()"<<endl;
    }
};

class Component : public ComponentBase {
public:
    Component() = default;

    void Init() {
        // Reset(); //不能访问基类的私有函数
        Clear();
        cout<<"Component::Init()"<<endl;
    }

    void InitInternal() override { // 重写虚函数不受访问权限控制
        cout<<"Component::InitInternal()"<<endl;
    }

    void Proc() override {
        cout<<"Component::Proc()"<<endl;
    }
};

class Example : public Component {
public:
    void Init() {
        cout<<"Example::Init()"<<endl;
    }

    void Proc() override {
        cout<<"Example::Proc()"<<endl;
    }
};

int main() {
    cout<<"==========main============"<<endl;
    Example ex;
    ex.Init();
    ex.Proc();
    ex.InitInternal();

    Component* co = &ex;
    co->Init(); // 非虚函数重写会出现问题,结果跟随基类指针声明的类型
    co->Proc();
    return 0;
}