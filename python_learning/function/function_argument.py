############################################
##ref: https://python3-cookbook.readthedocs.io/zh_CN/latest/c07/p01_functions_that_accept_any_number_arguments.html
#############################################

def avg(first, *rest):
    """
    为了能让一个函数接受任意数量的位置参数，可以使用一个*参数
    rest是由所有其他位置参数组成的元组。然后我们在代码中把它当成了一个序列来进行后续的计算。
    """
    return (first + sum(rest)) / (1 + len(rest))

def make_element(name, value, **attrs):
    """
    为了接受任意数量的关键字参数，使用一个以**开头的参数
    attrs是一个包含所有被传入进来的关键字参数的字典。
    """
    ## define function can accept more than one arguments
    ## the attrs received as a dictionary
    keyvals = ['%s="%s"' % item for item in attrs.items()]
    attr_str = ' '.join(keyvals)
    print(attr_str)

    for item in attrs.items():
        print(item)

def anyargs(*args, **kwargs):
    """
    如果你还希望某个函数能同时接受任意数量的位置参数和关键字参数，可以同时使用*和**
    使用这个函数时，所有位置参数会被放到args元组中，所有关键字参数会被放到字典kwargs中
    """
    print(args) # A tuple
    print(kwargs) # A dict

if __name__ == "__main__":
    avg(1, 2, 3, 4, 5)
    print("===============")
    make_element('item', 'Albatross', size='large', quantity=6)
    print("===============")
    anyargs(1, 2, 3, 4, size=12, length=10)