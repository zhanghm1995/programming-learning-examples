"""
https://www.zmonster.me/2016/03/09/numpy-slicing-and-indexing.html
"""

import numpy as np

def process():
    x = np.random.rand(4,3,2)
    print(x, x.shape)
    print(x.dtype)
    y = x[:,:1]
    print(y, y.shape)
    print(len(x[1]))

    mi = np.min(x, 1)
    print(mi, mi.shape)

def count_matrix_elements():
    """
    https://note.nkmk.me/en/python-numpy-count/
    """
    x = np.random.rand(5,3) * 2.0
    print(x, x.shape)
    
    y = x<1.0
    print(y, y.shape, y.dtype)

    print(np.count_nonzero(x<1.0))
    print(np.count_nonzero(x<1.0, axis=1))

def partial_operate_matrix():
    '''
    https://note.nkmk.me/en/python-numpy-where/
    '''
    ## Assigne same value for one condition
    arr = np.random.rand(5,5)
    mask = arr>0.5
    arr[mask] = 0

    ## Using np.where
    a = np.arange(9).reshape((3, 3))
    print(a)
    b = np.where(a<4)
    print(b, type(b))
    print(np.where(a < 4, -1, 100))

    c = np.where(a<4, a*3, a)
    print(c)

    b = a[:,1]<4
    print(b)
    c = np.where(b, a*3, a)
    print(c)
    

if __name__ == '__main__':
    process()
    count_matrix_elements()
    partial_operate_matrix()