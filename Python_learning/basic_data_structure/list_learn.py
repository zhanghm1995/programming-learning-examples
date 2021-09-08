## All list related operations
import fire

def create_list():
    a = [1, 2, 3, 4, 5]

    ## Create list has repeat values
    num, N = 3, 10
    b = N * [num]
    print(b)

def add_list():
    list_a = list(range(0, 5))
    list_b = list(range(5, 10))
    print(list_a)
    print(list_b)
    
    ## Using + would expand the list
    list_c = list_a + list_b
    print(list_c) # [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

    ## Using append would get a list of list
    list_c = []
    list_c.append(list_a)
    list_c.append(list_b)
    print(list_c) # [[0, 1, 2, 3, 4], [5, 6, 7, 8, 9]]


def slice_list():
    list_a = list(range(10))
    ## 每隔两个取一个元素
    res_a  = list_a[::2]  # [0, 2, 4, 6, 8]
    print(res_a)
    ## 删除最后一个元素
    res_a = list_a[:-1]
    print(res_a)
    ## 元素倒序, 注意和删除最后一个元素写法的区别
    res_a = list_a[::-1]
    print(res_a)



if __name__ == "__main__":
    slice_list()
    add_list()
    fire.Fire()