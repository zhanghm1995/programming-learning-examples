"""
ref: https://www.runoob.com/python3/python3-class.html
Reference: https://realpython.com/python-super/#super-in-multiple-inheritance
"""

from easydict import EasyDict


class BaseExample(object):
    def __init__(self) -> None:
        print("This is in BaseExample class")

    def forward(self):
        print("This is in BaseExample class")
    

class DerivedExample(BaseExample):
    def __init__(self) -> None:
        print("============__init__==============")
        # super(DerivedExample, self).__init__()
        # super().__init__()
        super(DerivedExample).__init__()
        print("This is in DerivedExample class")
    
    def forward(self):
        print("============forward===============")
        super().forward()
        print("This is in DerivedExample class")
    
    def area(self):
        print("compute the area in DerivedExample class")

class DerivedExample2(DerivedExample):
    def __init__(self) -> None:
        print("============__init__==============")
        super(DerivedExample, self).__init__()
        # super().__init__()
        print("This is in DerivedExample class")

        super(DerivedExample, self).area()
    
    def forward(self):
        print("============forward===============")
        super().forward()
        print("This is in DerivedExample class")


class MemberExample(object):
    def __init__(self):
        self.config = EasyDict(lr_scheduler="gamm")

    def test(self):
        if hasattr(self.config, "lr_scheduler"):
            print("has defined lr_scheduler")
        else:
            print("not defined lr_scheduler")


class BaseDataset(object):

    def __init__(self):
        print("__init__: This is in BaseDataset class")

    def evaluate(self, pred, gt):
        print("This is in BaseDataset class evaluate")


class DerivedDataset(BaseDataset):

    def __init__(self):
        super().__init__()
        print("__init__: This is in DerivedDataset class")
    
    def evaluate(self, pred):
        print("This is in DerivedDataset class evaluate")


def main1():
    member_example = MemberExample()
    member_example.test()
    exit(0)

    derived_example = DerivedExample()

    derived_example.forward()


def main2():
    """
    Validate the same name function overloading in Python class.
    """
    dataset = DerivedDataset()
    dataset.evaluate(1)


if __name__ == "__main__":
    main2()    