import numpy as np


def define_structed_array():
    """Methods to define structured arrays.
    """
    ## Method1
    cloud_arr = np.zeros(5, dtype=([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)]))
    
    print('cloud_arr:', len(cloud_arr), cloud_arr)
    
    ## Method2
    field_properties = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.int32)]
    cloud_arr2 = np.zeros(5, dtype=(field_properties))
    print('cloud_arr2:', len(field_properties),  cloud_arr2)
    print(type(cloud_arr2))


def assign_structured_array():
    # create a numpy array data
    data = np.random.randint(0,10,(6,5)) 
    data = data.astype(np.float32)
    print(data, data.dtype)

    # Init a structured array
    field_properties = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.int32)]
    cloud_arr2 = np.zeros(6, dtype=(field_properties))
    print('cloud_arr2:', len(field_properties),  cloud_arr2)

    for i, name in enumerate(cloud_arr2.dtype.names):
        cloud_arr2[name] = data[:, i]
    print(cloud_arr2)



# assert len(fields_tuple) == 4,  f'The size of {len(fields_tuple)} != 5!'

if __name__ == '__main__':
    define_structed_array()

    assign_structured_array()
    