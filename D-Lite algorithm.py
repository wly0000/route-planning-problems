#!/usr/bin/env python
# coding: utf-8

# In[12]:


import numpy as np
import heapq

from numpy.random import random_integers as rnd, randint
import matplotlib.pyplot as plt


class Element:
    def __init__(self, key, value1, value2):
        self.key = key
        self.value1 = value1
        self.value2 = value2

    def __eq__(self, other):
        return np.sum(np.abs(self.key - other.key)) == 0

    def __ne__(self, other):
        return self.key != other.key

    def __lt__(self, other):
        return (self.value1, self.value2) < (other.value1, other.value2)

    def __le__(self, other):
        return (self.value1, self.value2) <= (other.value1, other.value2)

    def __gt__(self, other):
        return (self.value1, self.value2) > (other.value1, other.value2)

    def __ge__(self, other):
        return (self.value1, self.value2) >= (other.value1, other.value2)


class DStarLitePlanning:
    def __init__(self, r_map, sx, sy, gx, gy):
        self.start = np.array([sx, sy])
        self.goal = np.array([gx, gy])
        self.k_m = 0
        self.rhs = np.ones((len(r_map), len(r_map[0]))) * np.inf
        self.g = self.rhs.copy()
        self.global_map = r_map
        self.sensed_map = np.zeros((len(r_map), len(r_map[0])))
        self.rhs[self.goal[0], self.goal[1]] = 0
        self.queue = []
        node = Element(self.goal, *self.CalculateKey(self.goal))
        heapq.heappush(self.queue, node)

    def CalculateKey(self, node):
        key = [0, 0]
        key[0] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]]) + self.h_estimate(self.start,
                                                                                             node) + self.k_m
        key[1] = min(self.g[node[0], node[1]], self.rhs[node[0], node[1]])
        return key

    def UpdateVertex(self, u):
        plt.plot()
        if np.sum(np.abs(u - self.goal)) != 0:
            s_list = self.succ(u)
            min_s = np.inf
            for s in s_list:
                if self.cost(u, s) + self.g[s[0], s[1]] < min_s:
                    min_s = self.cost(u, s) + self.g[s[0], s[1]]
            self.rhs[u[0], u[1]] = min_s
        if Element(u, 0, 0) in self.queue:
            self.queue.remove(Element(u, 0, 0))
            heapq.heapify(self.queue)
        if self.g[u[0], u[1]] != self.rhs[u[0], u[1]]:
            heapq.heappush(self.queue, Element(u, *self.CalculateKey(u)))

    def ComputeShortestPath(self):
        while len(self.queue) > 0 and                 heapq.nsmallest(1, self.queue)[0] < Element(self.start, *self.CalculateKey(self.start)) or                 self.rhs[self.start[0], self.start[1]] != self.g[self.start[0], self.start[1]]:
            k_old = heapq.nsmallest(1, self.queue)[0]
            u = heapq.heappop(self.queue).key
            if k_old < Element(u, *self.CalculateKey(u)):
                heapq.heappush(self.queue, Element(u, *self.CalculateKey(u)))
            elif self.g[u[0], u[1]] > self.rhs[u[0], u[1]]:
                self.g[u[0], u[1]] = self.rhs[u[0], u[1]]
                s_list = self.succ(u)
                for s in s_list:
                    self.UpdateVertex(s)
            else:
                self.g[u[0], u[1]] = np.inf
                s_list = self.succ(u)
                s_list.append(u)
                for s in s_list:
                    self.UpdateVertex(s)

    # fetch successors and predessors 
    def succ(self, u):
        s_list = [np.array([u[0] - 1, u[1] - 1]), np.array([u[0] - 1, u[1]]), np.array([u[0] - 1, u[1] + 1]),
                  np.array([u[0], u[1] - 1]), np.array([u[0], u[1] + 1]), np.array([u[0] + 1, u[1] - 1]),
                  np.array([u[0] + 1, u[1]]), np.array([u[0] + 1, u[1] + 1])]
        row = len(self.global_map)
        col = len(self.global_map[0])
        real_list = []
        for s in s_list:
            if 0 <= s[0] < row and 0 <= s[1] < col:
                real_list.append(s)
        return real_list

    # heuristic estimation
    def h_estimate(self, s1, s2):
        return np.linalg.norm(s1 - s2)

    # calculate cost between nodes
    def cost(self, u1, u2):
        if self.sensed_map[u1[0], u1[1]] == np.inf or self.sensed_map[u2[0], u2[1]] == np.inf:
            return np.inf
        else:
            return self.h_estimate(u1, u2)

    def sense(self, range_s):
        real_list = []
        row = len(self.global_map)
        col = len(self.global_map[0])
        for i in range(-range_s, range_s + 1):
            for j in range(-range_s, range_s + 1):
                if 0 <= self.start[0] + i < row and 0 <= self.start[1] + j < col:
                    if not (i == 0 and j == 0):
                        real_list.append(np.array([self.start[0] + i, self.start[1] + j]))
        return real_list


def Main(global_map, gx, gy, sx, sy):
    node = DStarLitePlanning(global_map, sx, sy, gx, gy)
    last = node.start
    last = ScanAndUpdate(node, last)
    node.ComputeShortestPath()
    while np.sum(np.abs(node.start - node.goal)) != 0:
        s_list = node.succ(node.start)
        min_s = np.inf
        for s in s_list:
            plt.plot(s[0],s[1], 'xy')
            if node.cost(node.start, s) + node.g[s[0], s[1]] < min_s:
                min_s = node.cost(node.start, s) + node.g[s[0], s[1]]
                temp = s
        node.start = temp.copy()
        print(node.start[0], node.start[1])
        plt.plot(node.start[0], node.start[1], '.b')
        last = ScanAndUpdate(node, last)
        plt.pause(0.1)

def ScanAndUpdate(ds, last):
    s_list = ds.sense(3)
    flag = True
    for s in s_list:
        if ds.sensed_map[s[0], s[1]] != ds.global_map[s[0], s[1]]:
            flag = False
            break
    if flag == False:
        ds.k_m += ds.h_estimate(last, ds.start)
        last = ds.start.copy()
        for s in s_list:
            if ds.sensed_map[s[0], s[1]] != ds.global_map[s[0], s[1]]:
                plt.plot(s[0],s[1], 'xr')
                ds.sensed_map[s[0], s[1]] = ds.global_map[s[0], s[1]]
                ds.UpdateVertex(s)
        ds.ComputeShortestPath()
    return last


def sense(self, range_s):
        real_list = []
        row = len(self.global_map)
        col = len(self.global_map[0])
        for i in range(-range_s, range_s + 1):
            for j in range(-range_s, range_s + 1):
                if 0 <= self.start[0] + i < row and 0 <= self.start[1] + j < col:
                    if not (i == 0 and j == 0):
                        real_list.append(np.array([self.start[0] + i, self.start[1] + j]))
        return real_list


def Main(global_map, gx, gy, sx, sy):
    node = DStarLitePlanning(global_map, sx, sy, gx, gy)
    last = node.start
    last = ScanAndUpdate(node, last)
    node.ComputeShortestPath()
    while np.sum(np.abs(node.start - node.goal)) != 0:
        s_list = node.succ(node.start)
        min_s = np.inf
        for s in s_list:
            plt.plot(s[0],s[1], 'xy')
            if node.cost(node.start, s) + node.g[s[0], s[1]] < min_s:
                min_s = node.cost(node.start, s) + node.g[s[0], s[1]]
                temp = s
        node.start = temp.copy()
        print(node.start[0], node.start[1])
        plt.plot(node.start[0], node.start[1], '.b')
        last = ScanAndUpdate(node, last)
        plt.pause(0.1)

def ScanAndUpdate(ds, last):
    s_list = ds.sense(3)
    flag = True
    for s in s_list:
        if ds.sensed_map[s[0], s[1]] != ds.global_map[s[0], s[1]]:
            flag = False
            break
    if flag == False:
        ds.k_m += ds.h_estimate(last, ds.start)
        last = ds.start.copy()
        for s in s_list:
            if ds.sensed_map[s[0], s[1]] != ds.global_map[s[0], s[1]]:
                plt.plot(s[0],s[1], 'xr')
                ds.sensed_map[s[0], s[1]] = ds.global_map[s[0], s[1]]
                ds.UpdateVertex(s)
        ds.ComputeShortestPath()
    return last
from time import time
import numpy as np

def print_map(global_map, path):
    for i, row in enumerate(global_map):
        for j, cell in enumerate(row):
            if (i, j) in path:
                print("*", end=" ")  # 路径上的点用 "*" 表示
            elif cell == 0:
                print(".", end=" ")  # 0 表示可通行区域，用 "." 表示
            else:
                print("#", end=" ")  # 1 表示障碍物区域，用 "#" 表示
        print()

def print_path(path):
    #print("路径坐标：")
    for point in path:
        print(point)
    #print("路径连续线：")
    for i in range(len(path)-1):
        print(path[i], "->", path[i+1])

def DStarLite(global_map, gx, gy, sx, sy):
    # D*Lite算法的实现
    # ...

    path = []   # 存储最佳路径的坐标
    # 进行路径追踪，将路径存储到path列表中
    
    # 返回最佳路径的坐标
    return path
from time import time
time1 = time()
if __name__ == "__main__":
    sx = 1
    sy = 2
    gx = 39
    gy = 38
    global_map = np.zeros((40, 40))  # 假设地图尺寸为10x10

    # 生成6个随机障碍物点的坐标
    obstacles = []
    num_obstacles = 6
    min_x = 1
    min_y = 1
    max_x = 39
    max_y = 39

    for _ in range(num_obstacles):
        x = np.random.randint(min_x, max_x+1)
        y = np.random.randint(min_y, max_y+1)
        obstacles.append((x, y))

    # 填充障碍物的位置
    for obstacle in obstacles:
        x, y = obstacle
        global_map[x][y] = 1

    # 输出地图和障碍物点的坐标
    #print("地图：")
    #print_map(global_map, [])

    for obstacle in obstacles:
        print("障碍物点坐标：", obstacle)

    # 调用DStarLite函数，传递地图矩阵、起点和终点的坐标作为参数
    path = DStarLite(global_map, gx, gy, sx, sy)

    # 输出最终传递的地图矩阵和路径
    print("最终传递的地图矩阵：")
    print_map(global_map, path)
    print_path(path)
from time import time
time2 = time()
print(f"总用时为{time2-time1}s")


# In[ ]:




