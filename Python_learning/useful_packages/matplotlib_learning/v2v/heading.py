"""
    Using for heading analysis
"""

import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties as FP
import numpy
import numpy as np


efp = FP('Times New Roman',size=15)


class TrackingEvaluation:
    def __init__(self, gt_filename, tr_filename, start, end):
        self.start = start
        self.end = end

        #### load groundtruth txt file
        gt = np.loadtxt(gt_filename)
        self.gt_pos_x = gt[self.start:self.end,1]
        self.gt_pos_y = gt[self.start:self.end,2]
        self.gt_velocity = (gt[self.start:self.end,5])
        self.gt_heading = gt[self.start:self.end,4]*180/3.141592654
        
        ### load track results
        track = np.loadtxt(tr_filename)
        self.tr_pos_x = track[self.start:self.end,1]
        self.tr_pos_y = track[self.start:self.end,2]
        self.tr_v = track[self.start:self.end,3]*3.6
        self.tr_heading = track[self.start:self.end,4]*180/3.141592654

        self.x_range = np.arange(len(self.gt_heading))
    
    def draw_position(self):
        plt.figure("position")
        plt.grid(linestyle="-.")
        plt.plot(self.gt_pos_x, self.gt_pos_y, color = "red", marker="*", label="GT")
        plt.plot(self.tr_pos_x, self.tr_pos_y, color = "blue", label="Track")
        plt.xlabel("X Position [m]", fontsize=15, fontproperties=efp)
        plt.ylabel("Y Position [m]", fontsize=15, fontproperties=efp)
        ax = plt.gca()
        plt.legend(fontsize=18)
        plt.tick_params(labelsize=15)
        plt.show()
    def draw_heading(self):
        plt.figure("yaw")
        plt.grid(linestyle="-.")
        plt.plot(self.x_range, self.gt_heading, color = "red", marker="*", label="GT")
        plt.plot(self.x_range, self.tr_heading, color = "blue", label="Track")
        # plt.title("UKF States")
        plt.xlabel("Frames", fontsize=12, fontproperties=efp)
        plt.ylabel("Yaw [rad]", fontsize=12, fontproperties=efp)
        ax = plt.gca()
        plt.legend(fontsize=15)
        plt.tick_params(labelsize=15)
        plt.show()

    def draw_heading_new(self):
        # Saving data
        # numpy.savetxt("yaw.txt", self.tr_heading)
        
        gt_new  = np.loadtxt("yaw.txt")
        mean = np.mean(gt_new[0:35,])
        print(mean)
        plt.figure("yaw")
        plt.grid(linestyle="-.")
        # plt.plot(self.x_range, self.gt_heading, color = "red", marker="*", label="GT")
        plt.plot(self.x_range, gt_new, color = "red", marker="*", label="GT")
        plt.plot(self.x_range, self.tr_heading, color = "blue", linewidth = 1.8, label="Track")
        # plt.title("UKF States")
        plt.xlabel("Frames", fontsize=15, fontproperties=efp)
        plt.ylabel("Yaw [degree]", fontsize=15, fontproperties=efp)
        ax = plt.gca()
        plt.legend(fontsize=18)
        plt.tick_params(labelsize=12)
        plt.show()

    def draw_velocity(self):
        # Saving data
        # numpy.savetxt("v.txt", self.tr_v)
        
        tr_new  = np.loadtxt("v_1008.txt")

        plt.figure("volocity")
        plt.grid(linestyle="-.")
        plt.plot(self.x_range, self.gt_velocity/3.6, color = "red", marker="*", label="GT")
        # plt.plot(self.x_range, self.tr_v, color = "blue", label="Track")
        plt.plot(self.x_range, tr_new, color = "blue", label="Track")
        # plt.title("UKF States")
        plt.xlabel("Frames", fontsize=15, fontproperties=efp)
        plt.ylabel("Velocity [km/h]", fontsize=15, fontproperties=efp)
        ax = plt.gca()
        plt.legend(fontsize=18)
        plt.tick_params(labelsize=12)
        plt.show()

    def draw_velocity_origin(self):
        plt.figure("volocity")
        plt.grid(linestyle="-.")
        plt.plot(self.x_range, self.gt_velocity, color = "red", marker="*", label="GT")
        # plt.plot(self.x_range, self.tr_v, color = "blue", label="Track")
        # self.tr_v[0] = self.gt_velocity[0]
        plt.plot(self.x_range, self.tr_v, color = "blue", label="Track")
        # plt.title("UKF States")
        plt.xlabel("Frames", fontsize=15, fontproperties=efp)
        plt.ylabel("Velocity [km/h]", fontsize=15, fontproperties=efp)
        ax = plt.gca()
        plt.legend(fontsize=18)
        plt.tick_params(labelsize=15)
        plt.show()


if __name__ == '__main__':
    data = 1008
    if (data == 1008):
        tr = TrackingEvaluation("1008_groundtruth.txt", "1008_track_results.txt", 121, 390)
        # tr.draw_heading()
        # tr.draw_heading_new()
        # tr.draw_velocity()
        tr.draw_position()
    elif (data == 1020):
        tr = TrackingEvaluation("1020_groundtruth.txt", "1020_track_results.txt", 0, 371)
        # tr.draw_heading()
        tr.draw_velocity_origin()
    elif (data == 1024):
        tr = TrackingEvaluation("1024_groundtruth.txt", "1024_track_results.txt", 0, 298)
        # tr.draw_heading()
        tr.draw_velocity_origin()
        