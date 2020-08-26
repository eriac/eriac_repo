"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time

class Node:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.parent_id = -1
        self.base_point = -1
    def __repr__(self):
        return "<Node x:%f, y:%f, parent:%u>" % (self.x, self.y, self.parent_id)

class Obstacle:
    def __init__(self):
        None

    def checkClearLine(self, a, b):
        resolution = 0.025
        dist = self.dist(a,b)
        num = int(dist / resolution)
        vx = b.x - a.x
        vy = b.y - a.y
        for i in range(num):
            px = a.x + vx * i / num
            py = a.y + vy * i / num
            if not self.checkClearPoint(Node(px, py)):
                return False
        return True

    def checkClearPoint(self, a):
        if 2.0 < a.x and a.x < 4.0 and -4.0 < a.y and a.y < 4.0:
            return False
        elif 6.0 < a.x and a.x < 8.0 and 6.0 < a.y and a.y < 14.0:
            return False
        else:
            return True

    def dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
    
    def getPatches(self):
        p1 = patches.Rectangle(xy=(2, -4), width=2.0, height=8.0)
        p2 = patches.Rectangle(xy=(6, 6), width=2.0, height=8.0)
        return [p1, p2]

class RRT:
    def __init__(self, area_min = (-2, -2), area_max = (12, 12)):
        self.nodes = []
        self.area_min = area_min
        self.area_max = area_max

    def getRandam(self):
        x = random.uniform(self.area_min[0], self.area_max[0])
        y = random.uniform(self.area_min[1], self.area_max[1])
        return Node(x,y)

    def getNearest(self, target):
        min_dist = 10000000.0
        min_index = -1
        for i in range(len(self.nodes)):
            n = self.nodes[i]
            dist = self.dist(target, n)
            if dist < min_dist:
                min_dist = dist
                min_index = i
        return min_index, dist

    def dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

    def plan(self, start, goal, obstacle):
        self.nodes.append(Node(start[0],start[1]))
        for i in range(300):
            goal_bias = not (i % 10)
            
            if goal_bias:
                rand_node = Node(goal[0], goal[1])
            else:
                rand_node = self.getRandam()

            rand_node.parent_id, dist = self.getNearest(rand_node)
            if dist < 0.1:
                continue
            if not obstacle.checkClearLine(self.nodes[rand_node.parent_id], rand_node):
                continue


            self.nodes.append(rand_node)
            if goal_bias:
                break

        path = [self.nodes[-1]]
        target_id = len(self.nodes)-1
        while True:
            child = self.nodes[target_id]
            target_id = child.parent_id
            if(target_id < 0):
                break
            parent = self.nodes[target_id]
            path.append(parent)

        path.reverse()
        return path

class Smoother:
    def __init__(self):
        None

    def get_path_length(self, path):
        le = 0
        for i in range(len(path) - 1):
            dx = path[i + 1].x - path[i].x
            dy = path[i + 1].y - path[i].y
            d = math.sqrt(dx * dx + dy * dy)
            le += d
        return le


    def get_target_point(self, path, targetL):
        le = 0
        ti = 0
        lastPairLen = 0
        for i in range(len(path) - 1):
            dx = path[i + 1].x - path[i].x
            dy = path[i + 1].y - path[i].y
            d = math.sqrt(dx * dx + dy * dy)
            le += d
            if le >= targetL:
                ti = i
                lastPairLen = d
                break

        partRatio = (le - targetL) / lastPairLen

        x = path[ti].x + (path[ti + 1].x - path[ti].x) * partRatio
        y = path[ti].y + (path[ti + 1].y - path[ti].y) * partRatio
        # print(x ,"=",path[ti].x,"+ (",path[ti + 1].x, "-", path[ti].x, ") *", partRatio)
        output = Node(x, y)
        output.base_point = ti
        return output


    def line_collision_check(self, first, second, obstacleList):
        # Line Equation

        x1 = first[0]
        y1 = first[1]
        x2 = second[0]
        y2 = second[1]

        try:
            a = y2 - y1
            b = -(x2 - x1)
            c = y2 * (x2 - x1) - x2 * (y2 - y1)
        except ZeroDivisionError:
            return False

        for (ox, oy, size) in obstacleList:
            d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
            if d <= size:
                return False

        return True  # OK


    def path_smoothing(self, path, obstacle, max_iter=200):

        for i in range(max_iter):
            # Sample two points
            le = self.get_path_length(path)

            if (i % 20) == 8:
                first = path[0]
                second = self.get_target_point(path, random.uniform(0, le))
            elif (i % 20) == 12:
                first = self.get_target_point(path, random.uniform(0, le))
                second = path[-1]
            else:
                pickPoints = [random.uniform(0, le), random.uniform(0, le)]
                pickPoints.sort()
                first = self.get_target_point(path, pickPoints[0])
                second = self.get_target_point(path, pickPoints[1])

            # if first.base_point <= 0 or second.base_point <= 0:
            #     continue

            # if (second.base_point + 1) > len(path):
            #     continue

            if second.base_point == first.base_point:
                continue

            # collision check
            if not obstacle.checkClearLine(first, second):
                continue

            # print(i,first, second)

            # Create New path
            newPath = []
            newPath.extend(path[:first.base_point + 1])
            newPath.append(first)
            newPath.append(second)
            newPath.extend(path[second.base_point + 1:])
            path = newPath

            error = False
            for j in range(len(path)-1):
                d = self.dist(path[j],path[j+1])
                if d < 0.00001:
                    error = True
            if error:
                print("position error")
                break

        return path

    def dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)

class PathSimplefy:
    def __init__(self):
        None

    def simplify(self, path, obstacle):
        for i in range(100):
            dist = []
            for j in range(len(smooth_path)-1):
                dx = smooth_path[j].x - smooth_path[j+1].x
                dy = smooth_path[j].y - smooth_path[j+1].y
                dist.append(math.sqrt(dx**2 +dy**2))
            
            for j in range(len(dist)):
                min_index = dist.index(min(dist))
                if 1 <= min_index < len(path)-1:
                    if obstacle.checkClearLine(path[min_index-1], path[min_index+1]):
                        # print("min["+str(min_index)+"] "+str(dist[min_index]))
                        new_path = []
                        new_path.extend(path[:min_index])
                        new_path.extend(path[min_index+1:])
                        path = new_path
                        break
                dist[min_index] = 10000
                
        return path

    def dist(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)


if __name__ == '__main__':
    start = (0, 0)
    goal = (10,10)
    obstacle = Obstacle()

    start_time = time.time()

    rrt = RRT()
    path = rrt.plan(start, goal, obstacle)

    smoother =Smoother()
    smooth_path = smoother.path_smoothing(path, obstacle)

    simple = PathSimplefy()
    simplify_path = simple.simplify(smooth_path, obstacle)

    end_time = time.time()
    print ("elapsed_time:{0}".format(end_time-start_time) + "[sec]")

    print(simplify_path)

    p_x = []
    p_y = []
    for i in range(len(path)):
        p_x.append(path[i].x)
        p_y.append(path[i].y)
    plt.plot(p_x, p_y)

    mp_x = []
    mp_y = []
    for i in range(len(smooth_path)):
        mp_x.append(smooth_path[i].x)
        mp_y.append(smooth_path[i].y)
    plt.plot(mp_x, mp_y)

    sp_x = []
    sp_y = []
    for i in range(len(simplify_path)):
        sp_x.append(simplify_path[i].x)
        sp_y.append(simplify_path[i].y)
    plt.plot(sp_x, sp_y)

    ax = plt.axes()
    patches = obstacle.getPatches()
    for p in patches:
        ax.add_patch(p)

    plt.show()
