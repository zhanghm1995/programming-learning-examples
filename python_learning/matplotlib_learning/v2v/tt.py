import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties as FP
import numpy as np

efp = FP('Times New Roman',size=13)

#### load groundtruth txt file
f = open("1020.txt", "r")
velocity = []
for line in f:
    line = line.strip()
    fields = line.split(" ")
    velocity.append(float(fields[5]))

plt.figure("velocity")
plt.grid(linestyle="-.")
x_heading = np.arange(len(velocity))
plt.plot(x_heading, velocity, color = "red", label="GT")
plt.title("UKF States")
plt.xlabel("Frames", fontsize=12, fontproperties=efp)
plt.ylabel("Yaw [rad]", fontsize=12, fontproperties=efp)
ax = plt.gca()

plt.show()
