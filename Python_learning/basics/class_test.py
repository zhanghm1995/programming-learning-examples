"""
ref: https://www.runoob.com/python3/python3-class.html
"""

from turtle import forward


class BaseExample:
    def __init__(self) -> None:
        pass

    def forward(self):
        print("This is in BaseExample class")
    

class DerivedExample(BaseExample):
    def __init__(self) -> None:
        super().__init__()
    
    def forward(self):
        super().forward()
        print("This is in DerivedExample class")
    

if __name__ == "__main__":
    derived_example = DerivedExample()

    derived_example.forward()