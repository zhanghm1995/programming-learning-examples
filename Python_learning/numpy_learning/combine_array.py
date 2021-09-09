import numpy as np


def stack_array():
    a = np.random.rand(5,3)
    b = np.random.rand(5,3)

    array_list = [a, b]
    c = np.stack(array_list, axis=0)
    print(c)
    print(c.shape)


if __name__ == "__main__":
    stack_array()