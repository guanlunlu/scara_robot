import math
# for visualization
# import matplotlib.pyplot as plt
# import pylab as pl
# from matplotlib import collections  as mc

class segment():
    def __init__(self, waypoint_list, res):
        self.waypoints_list = waypoint_list
        self.trajectory = []
        # step size (mm per step)
        self.get_trajectory(res)
        pass

    def distance(self, p1, p2):
        d_x = p1[0] - p2[0]
        d_y = p1[1] - p2[1]
        return math.sqrt(pow(d_x, 2) + pow(d_y, 2))

    def get_trajectory(self, res):
        for i in range(len(self.waypoints_list)-1):
            traj_len = 0
            v_x = self.waypoints_list[i+1][0] - self.waypoints_list[i][0]
            v_y = self.waypoints_list[i+1][1] - self.waypoints_list[i][1]
            d_ = self.distance(self.waypoints_list[i+1], self.waypoints_list[i])
            d_x = v_x/d_
            d_y = v_y/d_

            while ( res * traj_len < d_ ):
                x0 = self.waypoints_list[i][0]
                y0 = self.waypoints_list[i][1]
                self.trajectory.append([x0 + res*d_x*traj_len, y0 + res*d_y*traj_len])
                traj_len += 1
            self.trajectory.append(self.waypoints_list[i+1])
        self.trajectory.append(self.waypoints_list[-1])
        

class word_day():
    def __init__(self, origin, res):
        self.origin = origin
        [org_x, org_y] = origin
        # self.seg_1 = segment([[org_x, org_y], [org_x, org_y+50], [org_x+30, org_y+50], [org_x+30, org_y], [org_x, org_y]],res)
        # self.seg_2 = segment([[org_x, org_y+25], [org_x+30, org_y+25]], res)
        self.seg_1 = segment([[org_x, org_y], [org_x, org_y+40], [org_x+20, org_y+40], [org_x+20, org_y], [org_x, org_y]],res)
        self.seg_2 = segment([[org_x, org_y+20], [org_x+20, org_y+20]], res)
        self.seg_list = [self.seg_1, self.seg_2]
        pass

    def visualize(self):
        x_seq = []
        y_seq = []
        for seg in self.seg_list:
            for pt in seg.trajectory:
                x_seq.append(pt[0])
                y_seq.append(pt[1])

        plt.plot(x_seq, y_seq)
        plt.show()
        pass

class word_spring():
    def __init__(self, origin, res):
        self.origin = origin
        [org_x, org_y] = origin
        self.seg_1 = segment([[org_x-12, org_y+80], [org_x+32, org_y+80]], res)
        self.seg_2 = segment([[org_x-7, org_y+65], [org_x+27, org_y+65]], res)
        self.seg_3 = segment([[org_x-20, org_y+50], [org_x+40, org_y+50]], res)
        self.seg_4 = segment([[org_x+10, org_y+95],[org_x+10, org_y+85], [org_x+7, org_y+60],[org_x, org_y+40], [org_x-15, org_y+10]], res)
        self.seg_5 = segment([[org_x+18, org_y+50], [org_x+20, org_y+40], [org_x+35, org_y+10]], res)
        self.seg_6 = segment([[org_x, org_y], [org_x, org_y+40], [org_x+20, org_y+40], [org_x+20, org_y], [org_x, org_y]],res)
        self.seg_7 = segment([[org_x, org_y+20], [org_x+20, org_y+20]], res)
        
        self.seg_list = [self.seg_1, self.seg_2, self.seg_3, self.seg_4, self.seg_5, self.seg_6, self.seg_7]
        pass

    def visualize(self):
        lines = []
        for seg in self.seg_list:
            line = []
            for pt in seg.trajectory:
                line.append(tuple(pt))
            lines.append(line)
        lc = mc.LineCollection(lines, linewidths=2)
        fig, ax = pl.subplots()
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        plt.show()
        pass



if __name__ == '__main__':
    # day = word_day([-25,0], 1)
    # day.visualize()

    spring = word_spring([0, 0], 1)
    spring.visualize()
