'''
Copyright (c) 2025 by Haiming Zhang. All Rights Reserved.

Author: Haiming Zhang
Date: 2025-06-12 10:57:16
Email: haimingzhang@link.cuhk.edu.cn
Description: 
'''
import os  
import torch  
import time  
import torch.multiprocessing as mp  

# Define the function to be executed by each process  
def run_on_gpu(rank, world_size):  
    """  
    Function executed by each process  
    :param rank: The GPU ID of the current process  
    :param world_size: Total number of GPUs  
    """  
    # Set the GPU to be used by the current process  
    torch.cuda.set_device(rank)  

    # Initialize tensors  
    A = torch.rand((30000, 30000)).float().cuda()  
    B = torch.rand((30000, 30000)).float().cuda()  
    A[100, 100] = 0  
    B[200, 200] = 3423  
    dust_bin = []  

    print(f"Process {rank} started on GPU {rank}")  

    while True:  
        try:  
            # Perform computations  
            C = A + B  
            C.inverse()  
            del_len = len(dust_bin) // 2  
            newC = torch.clone(C)  
            dust_bin.append(newC)  
        except Exception as e:  
            # Clean up part of the memory  
            del dust_bin[:del_len]  
            time.sleep(5)  

        time.sleep(0.5)

# Main function  
if __name__ == "__main__":  
    # Total number of GPUs  
    world_size = 8  # Assume there are 8 GPUs  

    # Use torch.multiprocessing.spawn to launch multiple processes  
    mp.spawn(  
        run_on_gpu,  # Function executed by each process  
        args=(world_size,),  # Parameters passed to the function  
        nprocs=world_size,  # Number of processes to start (i.e., number of GPUs)  
        join=True  # Wait for all processes to complete  
    )