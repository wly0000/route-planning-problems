#!/usr/bin/env python
# coding: utf-8

# In[1]:


#A*串行分布算法
def a_star_search(start, end):
    # 待访问的格子
    open_list = []
    # 已访问的格子
    close_list = []
    # 把起点加入open_list
    open_list.append(start)
    # 主循环，每一轮检查一个当前方格节点
    while len(open_list) > 0:
        # 在open_list中查找 F值最小的节点作为当前方格节点
        current_grid = find_min_grid(open_list)
        # 当前方格节点从openList中移除
        open_list.remove(current_grid)
        # 当当前方格节点进入 closeList
        close_list.append(current_grid)
        # 找到所有邻近节点
        neighbors = find_neighbors(current_grid, open_list, close_list)
        for grid in neighbors:
            if grid not in open_list:
                # 邻近节点不在openList中，标记父亲、G、H、F，并放入openList
                grid.init_grid(current_grid, end)
                open_list.append(grid)
        # 如果终点在openList中，直接返回终点格子
        for grid in open_list:
            if (grid.x == end.x) and (grid.y == end.y):
                return grid
    # openList用尽，仍然找不到终点，说明终点不可到达，返回空
    return None

def find_min_grid(open_list=[]):
    if len(open_list) == 0:
        return None
    temp_grid = open_list[0]
    for grid in open_list:
        if grid.f < temp_grid.f:
            temp_grid = grid
    return temp_grid

def find_neighbors(grid, open_list=[], close_list=[]):
    grid_list = []
    if is_valid_grid(grid.x, grid.y - 1, open_list, close_list):
        grid_list.append(Grid(grid.x, grid.y - 1))
    if is_valid_grid(grid.x, grid.y + 1, open_list, close_list):
        grid_list.append(Grid(grid.x, grid.y + 1))
    if is_valid_grid(grid.x - 1, grid.y, open_list, close_list):
        grid_list.append(Grid(grid.x - 1, grid.y))
    if is_valid_grid(grid.x + 1, grid.y, open_list, close_list):
        grid_list.append(Grid(grid.x + 1, grid.y))
    return grid_list

def is_valid_grid(x, y, open_list=[], close_list=[]):
    # 是否超过边界
    if x < 0 or x >= len(MAZE) or y < 0 or y >= len(MAZE[0]):
        return False
    # 是否有障碍物
    if MAZE[x][y] == 1:
        return False
    # 是否已经在open_list中
    if contain_grid(open_list, x, y):
        return False
    # 是否已经在closeList中
    if contain_grid(close_list, x, y):
        return False
    return True

def contain_grid(grids, x, y):
    for grid in grids:
        if (grid.x == x) and (grid.y == y):
            return True
    return False

class Grid:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = 0
        self.g = 0
        self.h = 0
        self.parent = None

    def init_grid(self, parent, end):
        self.parent = parent
        if parent is not None:
            self.g = parent.g + 1
        self.h = abs(self.x - end.x) + abs(self.y - end.y)
        self.f = self.g + self.h

# 迷宫地图
MAZE =[[0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
       [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
       [0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
       [0., 0., 0., 0., 0., 0., 0., 0., 0., 1.],
       [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]
from time import time   
time1 = time()
# 设置起点和终点
start_grid_1 = Grid(1, 2)
start_grid_2 = Grid(1, 1)
end_grid_1 = Grid(9, 7)
end_grid_2 = Grid(8, 8)

# 搜索迷宫终点
result_grid_1 = a_star_search(start_grid_1, end_grid_1)
result_grid_2 = a_star_search(start_grid_2, end_grid_2)

# 回溯迷宫路径
path_1 = []
path_2 = []
while result_grid_1 is not None:
    path_1.append(Grid(result_grid_1.x, result_grid_1.y))
    result_grid_1 = result_grid_1.parent

while result_grid_2 is not None:
    path_2.append(Grid(result_grid_2.x, result_grid_2.y))
    result_grid_2 = result_grid_2.parent

print("path_1:")
# Reverse the order of the coordinates to go from start to end
path_1.reverse()
path_1_str = ', '.join([f'({coord.x}, {coord.y})' for coord in path_1])
print(path_1_str)

print("path_2:")
# Reverse the order of the coordinates to go from start to end
path_2.reverse()
path_2_str = ', '.join([f'({coord.x}, {coord.y})' for coord in path_2])
print(path_2_str)
from time import time   
time2 = time()
print(f"总用时为{time2-time1}s")


# In[ ]:




