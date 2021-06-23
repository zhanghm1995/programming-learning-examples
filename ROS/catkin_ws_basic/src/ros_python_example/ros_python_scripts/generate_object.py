import numpy as np
import random

X_LIM = (-30.0, 50.0)
Y_LIM = (-30.0, 30.0)

x = random.uniform(*X_LIM)
y = random.uniform(*Y_LIM)
print(x)

def gen_random_pt():
    x = random.uniform(*X_LIM)
    y = random.uniform(*Y_LIM)
    return x, y

def birth_new_pt(existing_pts, mininum_dist=6.0, num_iter=100):
    if len(existing_pts) == 0:
        x, y = gen_random_pt()
        new_pt = np.array([[x, y]])
        return new_pt
    
    is_find_new_pt = False
    for i in range(num_iter):
        x, y = gen_random_pt()
        new_pt = np.array([x, y])
        dist = np.linalg.norm(new_pt - existing_pts, axis=-1)
        if (dist < mininum_dist).any():
            continue
        else:
            is_find_new_pt = True
            break
    
    if is_find_new_pt:
        return np.vstack((existing_pts, new_pt))
    else:
        return existing_pts

def generate_data(num_pt):
    curr_pts = np.array([])
    for i in range(num_pt):
        curr_pts = birth_new_pt(curr_pts)
        assert curr_pts is not None
    return curr_pts
