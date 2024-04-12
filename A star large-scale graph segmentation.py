#!/usr/bin/env python
# coding: utf-8

# In[2]:


from joblib import Parallel, delayed
import numpy as np
from time import time 
from collections import defaultdict


def a_star_search_m(maze, boundary, start_list, end_list):
    """
    根据大图、分割的子图边界，计算多个入口点与出口点的路径长度
    返回：dict
    """
    results={}
    for start in start_list:
        for end in end_list:
            start_grid = Grid(start[0], start[1])
            end_grid = Grid(end[0],end[1])
            result_grid = a_star_search(maze, boundary, start_grid, end_grid)
            path = []
            while result_grid is not None:
                path.append(Grid(result_grid.x, result_grid.y))
                result_grid = result_grid.parent
            if len(path)>0:
                results[(start,end)]=len(path)
    return results
    
    
def a_star_search(maze, boundary, start, end):
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
        neighbors = find_neighbors(maze, boundary, current_grid, open_list, close_list)
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

def find_neighbors(maze, boundary, grid, open_list=[], close_list=[]):
    grid_list = []
    if is_valid_grid(maze, boundary, grid.x, grid.y - 1, open_list, close_list):
        grid_list.append(Grid(grid.x, grid.y - 1))
    if is_valid_grid(maze, boundary, grid.x, grid.y + 1, open_list, close_list):
        grid_list.append(Grid(grid.x, grid.y + 1))
    if is_valid_grid(maze, boundary, grid.x - 1, grid.y, open_list, close_list):
        grid_list.append(Grid(grid.x - 1, grid.y))
    if is_valid_grid(maze, boundary, grid.x + 1, grid.y, open_list, close_list):
        grid_list.append(Grid(grid.x + 1, grid.y))
    return grid_list

def is_valid_grid(maze, boundary, x, y, open_list=[], close_list=[]):
    # 是否超过边界
    if x < boundary[0][0] or x > boundary[1][0] or y < boundary[0][1] or y > boundary[1][1]:
        return False
    # 是否有障碍物
    if maze[x][y] == 1:
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


def djkstra(graph, start, end):
    """
    迪杰斯特拉算法汇总个子图寻路结果，找到最短路径
    """
    path_set = set()    # 已求的路径集合
    priority_dic = {}
    for k in graph.keys():
        priority_dic[k] = [9999, False, ""] # 权重表构建为一个3维数组，分别是：最小路径代价，是否计算过最小边，最小路径
    priority_dic[start][0] = 0

    # 判断权重表中所有路点是否添加完毕
    def isSelectAll():
        ret = True
        for val in priority_dic.values():
            if not val[1]:
                ret = False
                break
        return ret

    while not isSelectAll():
        find_point = start
        find_path = start
        min_distance = 9999
        for path in path_set:
            end_point = path[-1]
            path_distance = priority_dic[end_point][0]
            if path_distance < min_distance and not priority_dic[end_point][1]:
                find_point = end_point
                find_path = path
                min_distance = path_distance
        find_distance = priority_dic[find_point][0]
        neighbors = graph[find_point]
        for k in neighbors.keys():
            p = str(find_path) + "-" + str(k)
            weight = find_distance + neighbors[k]
            path_set.add(p)
            if weight < priority_dic[k][0]:
                priority_dic[k][0] = weight
                priority_dic[k][2] = p
        priority_dic[find_point][1] = True

    return priority_dic[end]
from time import time   
time1 = time()
if __name__ == '__main__':
    # Initialize the maze matrix
    MAZE = np.zeros((92, 77))

    # Define obstacle points
    obstacle_points = [
        (32, 25),
        (32, 26),
        (32, 27),
        (44, 38),
        (45, 37),
        (46, 37),
        (47, 36),
        (48, 36),
        (36, 40),
        (28, 38),
        (27, 38),
        (26, 39),
        (25, 39),
        (35, 36),
        (36, 37)
        
    ]

    # Add obstacles to the maze
    for obstacle_point in obstacle_points:
        x, y = obstacle_point
        MAZE[x, y] = 1  # Mark obstacle points as 1

    njobs = 4
    start = (0,0)
    end = (91,76)

    # 参数：[大图,子图边界(即左上角点与右下角点)，入口点列表，出口点列表]
    params = [
        [MAZE, [(0, 0), (45, 38)], [(0, 0)], [(0, 38), (45, 0), (45, 38)]],
        [MAZE, [(0, 38), (45, 76)], [(0, 38)], [(45, 38), (45, 76)]],
        [MAZE, [(45, 0), (91, 38)], [(45, 0)], [(45, 38), (91, 38)]],
        [MAZE, [(45, 38), (91, 76)], [(45, 38)], [(45, 76), (91, 38), (91, 76)]]
    ]
    
   # 各子图并行寻路，找到多条路径
    all_results = Parallel(n_jobs=njobs)(delayed(a_star_search_m)(*param) for param in params)

    # 整理结果 
    node=sorted(list(set([k for results in all_results for se in results.keys() for k in se])))
    node_id_dict=dict(zip(node,[str(i) for i in range(len(node))]))
    id_node_dict={v:k for k,v in node_id_dict.items()}
    graph = defaultdict(dict)
    for results in all_results:
        for se,v in results.items():
            graph[node_id_dict[se[0]]].update({node_id_dict[se[1]]:v}) 
    for oid in list(set(id_node_dict.keys())-set(graph.keys())):
        graph[oid]={}

    # 使用djkstra算法找到大图路径
    result = djkstra(graph, node_id_dict[start], node_id_dict[end])

    weight=result[0]
    path=result[2]
    path_str='-'.join([str(id_node_dict[i]) for i in path.split('-')])
    print('路径长度：',weight)
    print('路径：')
    print(path_str)
from time import time   
time2 = time()
print(f"总用时为{time2-time1}s")


# In[5]:


#庙算地图障碍物坐标，庙算地图为92*77大小
obstacle_points = [
        (32, 25),
        (32, 26),
        (32, 27),
        (44, 38),
        (45, 37),
        (46, 37),
        (47, 36),
        (48, 36),
        (36, 40),
        (28, 38),
        (27, 38),
        (26, 39),
        (25, 39),
        (35, 36),
        (36, 37)
        
    ]


# In[ ]:


#庙算地图障碍区坐标，庙算地图为92*77大小
obstacle_points = [
        (29, 32),
        (30, 32),
        (31, 32),
        (35, 41),
        (35, 42),
        (36, 43),
        (39, 32),
        (40, 32),
        (41, 31)   
    ]


# In[ ]:




