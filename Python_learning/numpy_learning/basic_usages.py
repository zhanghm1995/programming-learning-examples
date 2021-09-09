import numpy as np

def use_as_matrix():
    A = np.array([[1,2,3], [4,5,6],[7,8,9]])
    print('A=\n', type(A), A, A.shape)
    print(A[1,1])

    ## define column vector
    B = np.array([[1], [2], [3]])
    print(B, B.shape)

     ## define row vector
    C = np.array([[1, 2, 3]])
    print(C, C.shape)

def use_as_array():
    A = np.arange(1,10,1)
    print(A, A.shape)

    B = np.linspace(1, 10, 10)
    print(B, B.shape)

def permute_matrix():
    matrix_input = np.random.rand(10, 3)
    print(matrix_input, matrix_input.shape)

    matrix_permute = matrix_input[:, [0,2,1]]
    print(matrix_permute)

def combine_arrays():
    a = np.random.rand(3, 96, 96)
    b = np.random.rand(3, 96, 96)
    c = np.random.rand(3, 96, 96)
    d = np.random.rand(3, 96, 96)
    window = [a, b, c, d]

    window_np = np.asarray(window)
    window_np2 = window_np.copy()
    print(window_np.shape)

    x = np.concatenate([window_np, window_np2], axis=0)
    print(x.shape)

if __name__ == "__main__":
    use_as_matrix()
    print("================")
    use_as_array()
    print("================")
    permute_matrix()
    print("==============")
    combine_arrays()