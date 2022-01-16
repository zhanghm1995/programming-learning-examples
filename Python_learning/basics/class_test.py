"""
ref: https://www.runoob.com/python3/python3-class.html
"""

class MyClass:
    """一个简单的类实例"""

    def __init__(self):
        self.data = [1,2,3]
        self.i = 23

    i = 12345
    __weight = 12 # private member
    def hello_world(self):
        print(self.__weight)
        return 'hello world'

    def __len__(self):
        return len(self.data)

# 实例化类
x = MyClass()
 
# 访问类的属性和方法
print("MyClass 类的属性 data 为：", len(x))
print("MyClass 类的属性 i 为：", x.i)
print("MyClass 类的方法 f 输出为：", x.hello_world())