"""
ref: https://github.com/trekhleb/learn-python/blob/master/src/functions/test_function_definition.py
"""

def greet(name):
    return 'Hello, ' + name

# You can assign functions to variables
greet_someone = greet
assert greet_someone('John') == 'Hello, John'

# Define functions inside other functions.
def greet_again(name):
    def get_message():
        return 'Hello, '

    result = get_message() + name
    return result