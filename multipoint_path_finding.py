# coding=utf-8

import heapq
import os
import pickle
import math

from collections import deque
from osm2networkx import *


# 优先队列
class PriorityQueue(object):

    def __init__(self):
        self.queue = []
        self._len = 0

    def pop(self):
        self._len -= 1
        node = heapq.heappop(self.queue)
        n_node = list(node[:-2])
        n_node.append(node[-1])
        return n_node

    def remove(self, node):
        for i in range(self._len):
           if self.queue[i][-1][0] == node[-1][0]:
               self.queue.pop(i)
               self._len -= 1
               heapq.heapify(self.queue)
               break

    def __iter__(self):

        return iter(sorted(self.queue))

    def __str__(self):

        return 'PQ:%s' % self.queue

    def append(self, node):
        node = list(node)
        n_node = node[:-1]
        n_node.extend([self._len, node[-1]])
        self._len += 1
        heapq.heappush(self.queue, n_node)
        
    def __contains__(self, key):

        return key in [n[-1] for n in self.queue]

    def __eq__(self, other):

        return self.queue == other.queue

    def size(self):

        return len(self.queue)

    def clear(self):

        self.queue = []
        self._len = 0

    def top(self):

        return self.queue[0]

    def peek_priority(self):
        if self._len > 0:
            node = self.top()
            return node[0]
        else:
            return None

    def find_by_id(self, nodeId):
        for i in range(self._len):
            id = self.queue[i][-1][0]
            if id == nodeId:
                result = self.queue[i]
                return result
        return None


def breadth_first_search(graph, start, goal):
    if start == goal:
        return []
    queue = deque([(start, [start])]) # 创建双向队列并将出发节点作为广度优先搜索的起点
    visited_set = set([start])
    while queue: # 当队列不为空时
        node_tuple = queue.popleft()                            # 弹出队头元素作为当前节点
        neighbors = sorted(graph.neighbors(node_tuple[0]))      # 获取当前节点的所有邻节点，按字母顺序排序
        for x in neighbors:
            if x not in visited_set:                            # 判断是否探索过该邻节点
                visited_set.add(x)
                new_path = list(node_tuple[1])                  # 获取出发节点到当前节点的路径
                new_path.append(x)
                queue.append((x, new_path))                     # 加入新节点及初始节点到该节点的路径
                if x == goal:
                    return new_path                                 # 如果新节点为目标节点，返回路径

    return [start, goal]


def uniform_cost_search(graph, start, goal):
    if start == goal:
        return []
    priority_queue = PriorityQueue()                # 创建优先队列，第一优先度为权值，第二优先度为入队时的队列长度
    priority_queue.append([0, 0, (start, [start])]) # [路径耗费权值，队列长度，(节点，初始节点到该节点的路径)]
    visited_set = set([start])
    while priority_queue._len: # 当队列不为空时
        priority_node = priority_queue.pop()        # 弹出队头元素作为当前节点
        node = priority_node[-1][0]                 # 获取当前节点的Key
        node_path = priority_node[-1][1]            # 获取出发节点到当前节点的路径 
        path_cost = priority_node[0]                # 初始节点到当前节点的路径耗费的权值
        visited_set.add(node)
        if node == goal:
            return node_path                        # 如果当前节点为目标节点，返回路径  
        neighbors = sorted(graph.neighbors(node))   # 获取当前节点的所有邻节点，按字母顺序排序
        for x in neighbors:
            if x not in visited_set:                                   # 判断是否探索过该邻节点
                new_path = list(node_path)
                new_path.append(x)                                     # 初始节点到该节点的路径 
                new_path_cost = graph.get_edge_data(node, x)['weight'] # 当前节点到该邻节点的权值
                new_node = [path_cost + new_path_cost, priority_queue._len, (x, new_path)]
                pnode = priority_queue.find_by_id(x)
                if not pnode:
                    priority_queue.append(new_node)               # 队列加入新节点
                else:
                    if new_node[0] < pnode[0]:
                        priority_queue.remove(pnode)
                        priority_queue.append(new_node)


    return [start, goal]


def null_heuristic(graph, v, goal):

    return 0


def euclidean_dist_heuristic(graph, v, goal):
    current_node = graph.nodes[v]
    goal_node = graph.nodes[goal]

    # 获取节点位置
    current_position = current_node['pos'] 
    goal_position = goal_node['pos']  

    # 计算两节点的欧式距离
    distance = math.sqrt((current_position[0] - goal_position[0])**2 +
                     (current_position[1] - goal_position[1])**2)
    return distance


def a_star(graph, start, goal, heuristic=euclidean_dist_heuristic):
    if start == goal:
        return []
 
    priority_queue = PriorityQueue()                # 创建优先队列，第一优先度为权值+欧式距离，第二优先度为入队时的队列长度
    priority_queue.append([0, 0, (start, [start])]) # [路径耗费权值+该节点与目标节点的欧式距离，队列长度，(节点，初始节点到该节点的路径)]
    visited_set = set([start])
    while priority_queue._len: # 当队列不为空时
        priority_node = priority_queue.pop()        # 弹出队头元素作为当前节点
        node = priority_node[-1][0]                 # 获取当前节点的Key
        node_path = priority_node[-1][1]            # 获取出发节点到当前节点的路径 
        path_cost = priority_node[0] - heuristic(graph, node, goal) # 初始节点到当前节点的路径耗费的权值+欧氏距离
        visited_set.add(node)
        if node == goal:
            return node_path                        # 如果当前节点为目标节点，返回路径
        neighbors = sorted(graph.neighbors(node))   # 获取当前节点的所有邻节点，按字母顺序排序
        for x in neighbors:
            if x not in visited_set:                                   # 判断是否探索过该邻节点
                new_path = list(node_path)
                new_path.append(x)                                     # 初始节点到该节点的路径
                new_path_cost = graph.get_edge_data(node, x)['weight'] # 当前节点到该邻节点的权值
                heuristic_value = heuristic(graph, x, goal)
                new_node = [path_cost + new_path_cost + heuristic_value, priority_queue._len,(x, new_path)]
                priority_queue.append(new_node)                   # 队列加入新节点
 
    return [start, goal]


def update_node_if_needed(priority_queue, current_score_dictionary, node):
    node_id = node[-1][0]
    node_priority = node[0]
    found_node = priority_queue.find_by_id(node_id)       # 寻找具有相同Key的节点
    if node_priority < found_node[0]:
        priority_queue.remove(found_node)
        priority_queue.append(node)                       # 添加权值更低的节点
        current_score_dictionary[node_id] = node_priority # 更新访问节点字典的权值


# 计算路径耗费的权值
def path_cost_calculation(graph, path):
    path_length = len(path)
    if path_length == 1:
        return 0
    cost = 0
    for i in range(path_length):
        if i + 1 <= path_length - 1:
            cost += graph.get_edge_data(path[i], path[i+1])['weight']
    return cost


# 连接路径
def concat_paths(path1, path2, is_forward = True):
    if is_forward:           # 判断path1是否是以初始节点开头的路径列表
        path2.reverse()
        path1.extend(path2)
        return path1
    else:
        path1.reverse()
        path2.extend(path1)
        return path2


def bidirectional_ucs(graph, start, goal):
    if start == goal:
        return []

    forward_visited = {}                              # 前向访问节点字典(键为节点，值为初始节点到该节点的路径)
    backward_visited = {}                             # 后向访问节点字典(键为节点，值为目标节点到该节点的路径)
    score_forward_per_node = {}                       # 前向访问节点字典(键为节点，值为初始节点到该节点的权值)
    score_backward_per_node = {}                      # 后向访问节点字典(键为节点，值为目标节点到该节点的权值)
    
    frontier_forward = PriorityQueue()                # 前向待访问节点队列 
    frontier_backward = PriorityQueue()               # 后向待访问节点队列 
    frontier_forward.append([0, 0, (start, [start])]) # [路径耗费权值，队列长度，(节点，初始节点到该节点的路径)]
    frontier_backward.append([0, 0, (goal, [goal])])
    score_forward_per_node[start] = 0
    score_backward_per_node[goal] = 0

    found_paths = PriorityQueue()                     # 已经寻找到的路径(初始节点到目标节点)的队列
    best_possible_value = None                        # 初始节点到目标节点的可能的最小权值

    while frontier_backward._len and frontier_forward._len: # 当前向队列和后向队列不为空
        if best_possible_value: # 判断前向队列和后向队列中的元素的最小权值及欧氏距离之和是否大于等于路径可能的最小权值，若是，则不可能出现更小的权值了，结束循环
            top_forward = frontier_forward.top()
            top_backward = frontier_backward.top()
            if score_forward_per_node[top_forward[-1][0]] + score_backward_per_node[top_backward[-1][0]] >= best_possible_value:
                break
        if frontier_forward.peek_priority() <= frontier_backward.peek_priority(): # 选取两个队列中队头元素的权值更小的为当前队列
            queue_to_process = frontier_forward
            visited_set = forward_visited                       # 指定当前队列对应的访问节点字典(路径)
            other_visited_set = backward_visited                # 指定另一个队列对应的访问节点字典(路径)
            current_score_dictionary = score_forward_per_node   # 指定当前队列对应的访问节点字典(权值)
            is_forward = True                                   # 当前队列是否为前向队列
        else:
            queue_to_process = frontier_backward
            visited_set = backward_visited
            other_visited_set = forward_visited
            current_score_dictionary = score_backward_per_node
            is_forward = False
        priority_node = queue_to_process.pop()      # 弹出队头元素作为当前节点
        node = priority_node[-1][0]                 # 获取当前节点的Key
        node_path = priority_node[-1][1]            # 获取出发节点到当前节点的路径 
        path_cost = priority_node[0]                # 初始(目标)节点到当前节点的路径耗费的权值
        visited_set[node] = node_path               # 将当前节点加入已访问节点字典
        neighbors = sorted(graph.neighbors(node))   # 获取当前节点的所有邻节点，按字母顺序排序
        for x in neighbors:
            if not x in visited_set: # 判断该邻节点是否已经访问过          
                new_path = list(node_path)
                new_path.append(x)                                     # 初始(目标)节点到该节点的路径
                new_path_cost = graph.get_edge_data(node, x)['weight'] # 当前节点到该邻节点的权值
                new_path_cost += path_cost
                new_node = [new_path_cost, queue_to_process._len, (x, new_path) ]
                if not queue_to_process.find_by_id(x): # 判断该邻节点是否是已探索过的节点(即已加入队列中)
                    queue_to_process.append(new_node)            # 未探索过则加入队列
                    current_score_dictionary[x] = new_path_cost  # 添加相应权值
                else: # 如果队列中存在该邻节点但当前探索的路径权值更低，则进行更新 
                    update_node_if_needed(queue_to_process, current_score_dictionary, new_node)
                if x in other_visited_set: # 如果该邻节点已经在另一个访问方向被访问过，则说明找到了一条初始节点到目标节点的新路径
                    current_path = list(node_path)
                    other_path = list(other_visited_set[x])
                    found_path = concat_paths(current_path, other_path, is_forward) # 对新路径进行拼接
                    found_path_cost = path_cost_calculation(graph, found_path)                  # 计算新路径的权值
                    found_paths.append([found_path_cost, found_path])               # 将新路径加入到路径队列中
                    # 更新路径可能的最小权值
                    if (best_possible_value and best_possible_value > found_path_cost) or not best_possible_value:
                        best_possible_value = found_path_cost
    # 返回找到的所有路径中权值最小的
    if found_paths._len:
        return found_paths.pop()[1]
    else:
        return [start, goal]


def bidirectional_a_star(graph, start, goal,
                         heuristic=euclidean_dist_heuristic):
    if start == goal:
        return []

    forward_visited = {}                              # 前向访问节点字典(键为节点，值为初始节点到该节点的路径)
    backward_visited = {}                             # 后向访问节点字典(键为节点，值为目标节点到该节点的路径)
    score_forward_per_node = {}                       # 前向访问节点字典(键为节点，值为初始节点到该节点的权值)
    score_backward_per_node = {}                      # 后向访问节点字典(键为节点，值为目标节点到该节点的权值)
    
    frontier_forward = PriorityQueue()                # 前向待访问节点队列 
    frontier_backward = PriorityQueue()               # 后向待访问节点队列 
    frontier_forward.append([0, 0, (start, [start])]) # [路径耗费权值+欧氏距离，队列长度，(节点，初始节点到该节点的路径)]
    frontier_backward.append([0, 0, (goal, [goal])])
    score_forward_per_node[start] = 0
    score_backward_per_node[goal] = 0

    found_paths = PriorityQueue()                     # 已经寻找到的路径(初始节点到目标节点)的队列
    best_possible_value = None                        # 初始节点到目标节点的可能的最小权值
    while frontier_backward._len and frontier_forward._len: # 当前向队列和后向队列不为空
        if best_possible_value: # 判断前向队列和后向队列中的元素的最小权值之和是否大于等于路径可能的最小权值，若是，则不可能出现更小的权值了，结束循环
            top_forward = frontier_forward.top()
            top_backward = frontier_backward.top()
            if top_forward[0] - heuristic(graph, top_forward[-1][0], goal) + top_backward[0] - heuristic(graph, top_backward[-1][0], start) >= best_possible_value:
                break
        if frontier_forward.peek_priority() <= frontier_backward.peek_priority(): # 选取两个队列中队头元素的权值更小的为当前队列
            queue_to_process = frontier_forward
            visited_set = forward_visited                       # 指定当前队列对应的访问节点字典(路径)
            other_visited_set = backward_visited                # 指定另一个队列对应的访问节点字典(路径)
            current_score_dictionary = score_forward_per_node   # 指定当前队列对应的访问节点字典(权值)
            is_forward = True                                   # 当前队列是否为前向队列
            goal_node = goal                                    # 指定当前访问方向的终点为目标节点
        else:
            queue_to_process = frontier_backward
            visited_set = backward_visited
            other_visited_set = forward_visited
            current_score_dictionary = score_backward_per_node
            is_forward = False
            goal_node = start
        priority_node = queue_to_process.pop()      # 弹出队头元素作为当前节点
        node = priority_node[-1][0]                 # 获取当前节点的Key
        node_path = priority_node[-1][1]            # 获取出发节点到当前节点的路径 
        path_cost = priority_node[0] - heuristic(graph, node, goal_node) # 初始(目标)节点到当前节点的路径耗费的权值
        visited_set[node] = node_path               # 将当前节点加入已访问节点字典
        neighbors = sorted(graph.neighbors(node))   # 获取当前节点的所有邻节点，按字母顺序排序
        for x in neighbors:
            if x not in visited_set: # 如果该邻节点已经被探索过，则不更新它
                new_path = list(node_path)
                new_path.append(x)                                     # 初始(目标)节点到该节点的路径
                new_path_cost = graph.get_edge_data(node, x)['weight'] # 当前节点到该邻节点的权值
                heuristic_value = heuristic(graph, x, goal_node)       # 该节点与初始(目标)节点的欧氏距离
                new_path_cost += path_cost + heuristic_value
                new_node = [new_path_cost, queue_to_process._len, (x, new_path)]
                if not queue_to_process.find_by_id(x): # 判断该邻节点是否是已探索过的节点(即已加入队列中)
                    queue_to_process.append(new_node)              # 未探索过则加入队列
                    current_score_dictionary[x] = new_path_cost    # 添加相应权值
                elif new_path_cost < current_score_dictionary[x]:  # 如果队列中存在该邻节点但当前探索的路径权值更低，则进行更新 
                    visited_set[x] = new_path
                    queue_to_process.remove(x)
                    queue_to_process.append(new_node)
                    current_score_dictionary[x] = new_path_cost
                if x in other_visited_set: # 如果该邻节点已经在另一个访问方向被访问过，则说明找到了一条初始节点到目标节点的新路径
                    current_path = list(node_path)
                    other_path = list(other_visited_set[x])
                    found_path = concat_paths(current_path, other_path, is_forward) # 对新路径进行拼接
                    found_path_cost = path_cost_calculation(graph, found_path)      # 计算新路径的权值
                    found_paths.append([found_path_cost, found_path])               # 将新路径加入到路径队列中
                    # 更新路径可能的最小权值
                    if (best_possible_value and best_possible_value > found_path_cost) or not best_possible_value:
                        best_possible_value = found_path_cost

    # 返回找到的所有路径中权值最小的
    if found_paths._len:
        return found_paths.pop()[1]
    else:
        return [start, goal]


def construct_tridirectional_path(path1, path2):
    copied_path1 = list(path1)
    copied_path2 = list(path2)
    if copied_path1[0] == copied_path2[0]:     # 1 -> 2, 1 -> 3
        copied_path1.reverse()                 # 2 -> 1
        copied_path1.extend(copied_path2[1:])  # 2 -> 1 -> 3
        return copied_path1
    if copied_path1[-1] == copied_path2[-1]:   # 2 -> 1, 3 -> 1
        copied_path1.reverse()                 # 1 -> 2
        copied_path2.extend(copied_path1[1:])  # 3 -> 1 -> 2
        return copied_path2
    if copied_path1[0] == copied_path2[-1]:    # 1 -> 2, 3 -> 1
        copied_path2.extend(copied_path1[1:])  # 3 -> 1 -> 2
        return copied_path2
    if copied_path1[-1] == copied_path2[0]:    # 1 -> 2, 2 -> 3
        copied_path1.extend(copied_path2[1:])  # 1 -> 2 -> 3
        return copied_path1


def tridirectional_search(graph, goals):
    if goals[0] == goals[1] == goals[2]:
        return []
    # 如果三个节点中有两个节点相同，则返回它们中的一个与剩下的节点的路径即可
    if goals[0] == goals[1]:
        return bidirectional_ucs(graph, goals[0], goals[2])
    if goals[1] == goals[2]:
        return bidirectional_ucs(graph, goals[0], goals[1])
    if goals[0] == goals[2]:
        return bidirectional_ucs(graph, goals[1], goals[2])

    goal1_visited = {}                                    # 目标一出发访问节点字典(键为节点，值为初始节点到该节点的路径)
    goal2_visited = {}                                    # 目标二出发访问节点字典(键为节点，值为初始节点到该节点的路径)    
    goal3_visited = {}                                    # 目标三出发访问节点字典(键为节点，值为初始节点到该节点的路径)      

    score_goal1_per_node = {}                             # 目标一出发访问节点字典(键为节点，值为初始节点到该节点的权值)
    score_goal2_per_node = {}                             # 目标二出发访问节点字典(键为节点，值为初始节点到该节点的权值)
    score_goal3_per_node = {}                             # 目标三出发访问节点字典(键为节点，值为初始节点到该节点的权值)

    frontier_goal1 = PriorityQueue()                      # 目标一出发待访问节点队列        
    frontier_goal2 = PriorityQueue()                      # 目标二出发待访问节点队列   
    frontier_goal3 = PriorityQueue()                      # 目标三出发待访问节点队列
    frontier_goal1.append([0, 0, (goals[0], [goals[0]])]) # [路径耗费权值，队列长度，(节点，初始节点到该节点的路径)]
    frontier_goal2.append([0, 0, (goals[1], [goals[1]])])
    frontier_goal3.append([0, 0, (goals[2], [goals[2]])])

    score_goal1_per_node[goals[0]] = 0
    score_goal2_per_node[goals[1]] = 0
    score_goal3_per_node[goals[2]] = 0

    best_score_by_pair = {1+2: float('inf'), 1+3: float('inf'), 2+3: float('inf')}           # 目标节点间路径的可能的最小权值      
    found_paths_by_pair = {1+2: PriorityQueue(), 1+3: PriorityQueue(), 2+3: PriorityQueue()} # 已经寻找到的路径(x+y:目标x和目标y)的队列

    while frontier_goal1._len and frontier_goal2._len and frontier_goal3._len: # 当三个队列均不为空
        top_goal1 = frontier_goal1.peek_priority()
        top_goal2 = frontier_goal2.peek_priority()
        top_goal3 = frontier_goal3.peek_priority()
        # 判断相应队列中的元素的最小权值之和是否大于等于路径可能的最小权值，若是，则不可能出现更小的权值了，结束循环
        if top_goal1 + top_goal2 >= best_score_by_pair[3] and top_goal1 + top_goal3 >= best_score_by_pair[4] and top_goal2 + top_goal3 >= best_score_by_pair[5]:
            break
        # 选取三个队列中队头元素的权值更小的为当前队列
        if top_goal1 <= top_goal2 and top_goal1 <= top_goal3:
            queue_to_process = frontier_goal1                   
            visited_set = goal1_visited                                    # 指定当前队列对应的访问节点字典(路径)
            current_score_dictionary = score_goal1_per_node                # 指定当前队列对应的访问节点字典(权值)
            other_visited_sets = [(2, goal2_visited), (3, goal3_visited)]  # 指定另两个队列对应的访问节点字典(路径)
            direction = 1                                                  # 指定出发点的序号              
        elif top_goal2 <= top_goal1 and top_goal2 <= top_goal3:
            queue_to_process = frontier_goal2
            visited_set = goal2_visited
            current_score_dictionary = score_goal2_per_node
            other_visited_sets = [(1, goal1_visited), (3, goal3_visited)]
            direction = 2
        else:
            queue_to_process = frontier_goal3
            visited_set = goal3_visited
            current_score_dictionary = score_goal3_per_node
            other_visited_sets = [(1, goal1_visited), (2, goal2_visited)]
            direction = 3
        priority_node = queue_to_process.pop()      # 弹出队头元素作为当前节点
        node = priority_node[-1][0]                 # 获取当前节点的Key
        node_path = priority_node[-1][1]            # 获取出发节点到当前节点的路径 
        path_cost = priority_node[0]                # 目标i节点到当前节点的路径耗费的权值
        visited_set[node] = node_path               # 将当前节点加入已访问节点字典
        neighbors = sorted(graph.neighbors(node))   # 获取当前节点的所有邻节点，按字母顺序排序
        for x in neighbors:
            if not x in visited_set: # 判断该邻节点是否已经访问过     
                new_path = list(node_path)
                new_path.append(x)                                     # 目标i节点到该节点的路径           
                new_path_cost = graph.get_edge_data(node, x)['weight'] # 当前节点到该邻节点的权值
                new_path_cost += path_cost
                new_node = [new_path_cost, queue_to_process._len, (x, new_path) ]
                if not queue_to_process.find_by_id(x): # 判断该邻节点是否是已探索过的节点(即已加入队列中)
                    queue_to_process.append(new_node)            # 未探索过则加入队列
                    current_score_dictionary[x] = new_path_cost  # 添加相应权值
                else: # 如果队列中存在该邻节点但当前探索的路径权值更低，则进行更新 
                    update_node_if_needed(queue_to_process, current_score_dictionary, new_node)
                # 如果该邻节点已经在另两个访问方向的第一个被访问过，则说明找到了一条目标节点x到目标节点y的新路径
                if x in other_visited_sets[0][1]: 
                    current_path = list(node_path)
                    other_path = list(other_visited_sets[0][1][x])
                    found_path = concat_paths(current_path, other_path) # 对新路径进行拼接
                    found_path_cost = path_cost_calculation(graph, found_path)      # 计算新路径的权值
                    # 更新路径可能的最小权值
                    path_idx = direction + other_visited_sets[0][0]     # 对应的路径序号(x+y)       
                    best_possible_value = best_score_by_pair[path_idx]  # 该路径当前的最小权值
                    if best_possible_value and best_possible_value > found_path_cost:
                        # 更新最小权值，并将新路径加入到路径队列中
                        best_score_by_pair[path_idx] = found_path_cost  
                        found_paths_by_pair[path_idx].append([found_path_cost, found_path])
                # 如果该邻节点已经在另两个访问方向的第二个被访问过，则说明找到了一条目标节点x到目标节点y的新路径
                if x in other_visited_sets[1][1]: 
                    current_path = list(node_path)
                    other_path = list(other_visited_sets[1][1][x])
                    found_path = concat_paths(current_path, other_path) # 对新路径进行拼接
                    found_path_cost = path_cost_calculation(graph, found_path)      # 计算新路径的权值
                    # 更新路径可能的最小权值
                    path_idx = direction + other_visited_sets[1][0]     # 对应的路径序号(x+y)       
                    best_possible_value = best_score_by_pair[path_idx]  # 该路径当前的最小权值
                    if best_possible_value and best_possible_value > found_path_cost:
                        # 更新最小权值，并将新路径加入到路径队列中
                        best_score_by_pair[path_idx] = found_path_cost  
                        found_paths_by_pair[path_idx].append([found_path_cost, found_path])
    path1_2 = []
    path1_3 = []
    path2_3 = []
    # 获取目标x和目标y之间找到的所有路径中权值最小的路径
    if found_paths_by_pair[3]._len > 0:
        path1_2 = found_paths_by_pair[3].pop()[1]
    if found_paths_by_pair[4]._len > 0:
        path1_3 = found_paths_by_pair[4].pop()[1]
    if found_paths_by_pair[5]._len > 0:
        path2_3 = found_paths_by_pair[5].pop()[1]
    
    if path1_2 and path1_3 and path2_3: # 如果三个目标之间两两存在路径
        final_queue = PriorityQueue()                           # 创建一个队列用于路径排序
        pair1 = construct_tridirectional_path(path1_2, path1_3) # 拼接 1 -> 2 和 1 -> 3
        pair2 = construct_tridirectional_path(path1_3, path2_3)
        pair3 = construct_tridirectional_path(path1_2, path2_3)
        final_queue.append([path_cost_calculation(graph, pair1), pair1])    # 将连通三个目标的路径加入队列
        final_queue.append([path_cost_calculation(graph, pair2), pair2])
        final_queue.append([path_cost_calculation(graph, pair3), pair3])
        # 返回拼接的路径中权值最小的
        return final_queue.pop()[1]
    elif path1_2 and path1_3: # 如果只有目标1、2和目标1、3间存在路径，则返回拼接的路径即可             
        return construct_tridirectional_path(path1_2, path1_3)
    elif path1_2 and path2_3:
        return construct_tridirectional_path(path1_2, path2_3)
    elif path1_3 and path2_3:
        return construct_tridirectional_path(path1_3, path2_3)
    else: # 无法找到一条路径能够连通三个目标
        return [start, goal]

def tridirectional_upgraded(graph, goals, heuristic=euclidean_dist_heuristic, landmarks=None):
    if goals[0] == goals[1] == goals[2]:
        return []
    # 如果三个节点中有两个节点相同，则返回它们中的一个与剩下的节点的路径即可
    if goals[0] == goals[1]:
        return bidirectional_a_star(graph, goals[0], goals[2], heuristic)
    if goals[1] == goals[2]:
        return bidirectional_a_star(graph, goals[0], goals[1], heuristic)
    if goals[0] == goals[2]:
        return bidirectional_a_star(graph, goals[1], goals[2], heuristic)

    goal1_visited = {}                                    # 目标一出发访问节点字典(键为节点，值为初始节点到该节点的路径)
    goal2_visited = {}                                    # 目标二出发访问节点字典(键为节点，值为初始节点到该节点的路径)    
    goal3_visited = {}                                    # 目标三出发访问节点字典(键为节点，值为初始节点到该节点的路径)      

    score_goal1_per_node = {}                             # 目标一出发访问节点字典(键为节点，值为初始节点到该节点的权值)
    score_goal2_per_node = {}                             # 目标二出发访问节点字典(键为节点，值为初始节点到该节点的权值)
    score_goal3_per_node = {}                             # 目标三出发访问节点字典(键为节点，值为初始节点到该节点的权值)

    frontier_goal1 = PriorityQueue()                      # 目标一出发待访问节点队列        
    frontier_goal2 = PriorityQueue()                      # 目标二出发待访问节点队列   
    frontier_goal3 = PriorityQueue()                      # 目标三出发待访问节点队列
    frontier_goal1.append([0, 0, (goals[0], [goals[0]])]) # [路径耗费权值+欧氏距离，队列长度，(节点，初始节点到该节点的路径)]
    frontier_goal2.append([0, 0, (goals[1], [goals[1]])])
    frontier_goal3.append([0, 0, (goals[2], [goals[2]])])

    score_goal1_per_node[goals[0]] = 0
    score_goal2_per_node[goals[1]] = 0
    score_goal3_per_node[goals[2]] = 0

    best_score_by_pair = {1+2: float('inf'), 1+3: float('inf'), 2+3: float('inf')}           # 目标节点间路径的可能的最小权值      
    found_paths_by_pair = {1+2: PriorityQueue(), 1+3: PriorityQueue(), 2+3: PriorityQueue()} # 已经寻找到的路径(x+y:目标x和目标y)的队列

    while frontier_goal1._len and frontier_goal2._len and frontier_goal3._len: # 当三个队列均不为空
        top_goal1 = frontier_goal1.peek_priority() - (heuristic(graph,frontier_goal1.top()[-1][0], goals[1]) + heuristic(graph, frontier_goal1.top()[-1][0], goals[2]))/2
        top_goal2 = frontier_goal2.peek_priority() - (heuristic(graph,frontier_goal2.top()[-1][0], goals[0]) + heuristic(graph, frontier_goal2.top()[-1][0], goals[2]))/2
        top_goal3 = frontier_goal3.peek_priority() - (heuristic(graph,frontier_goal3.top()[-1][0], goals[0]) + heuristic(graph, frontier_goal3.top()[-1][0], goals[1]))/2
        # 判断相应队列中的元素的最小权值之和是否大于等于路径可能的最小权值，若是，则不可能出现更小的权值了，结束循环
        if top_goal1 + top_goal2 >= best_score_by_pair[3] and top_goal1 + top_goal3 >= best_score_by_pair[4] and top_goal2 + top_goal3 >= best_score_by_pair[5]:
            break
        # 选取三个队列中队头元素的权值更小的为当前队列
        top_goal1 = frontier_goal1.peek_priority()
        top_goal2 = frontier_goal2.peek_priority()
        top_goal3 = frontier_goal3.peek_priority()
        if top_goal1 <= top_goal2 and top_goal1 <= top_goal3:
            queue_to_process = frontier_goal1                   
            visited_set = goal1_visited                                    # 指定当前队列对应的访问节点字典(路径)
            current_score_dictionary = score_goal1_per_node                # 指定当前队列对应的访问节点字典(权值)
            other_visited_sets = [(2, goal2_visited), (3, goal3_visited)]  # 指定另两个队列对应的访问节点字典(路径)
            other_goals = [goals[1], goals[2]]                             # 其他目标
            direction = 1                                                  # 指定出发点的序号              
        elif top_goal2 <= top_goal1 and top_goal2 <= top_goal3:
            queue_to_process = frontier_goal2
            visited_set = goal2_visited
            current_score_dictionary = score_goal2_per_node
            other_visited_sets = [(1, goal1_visited), (3, goal3_visited)]
            other_goals = [goals[0], goals[2]]
            direction = 2
        else:
            queue_to_process = frontier_goal3
            visited_set = goal3_visited
            current_score_dictionary = score_goal3_per_node
            other_visited_sets = [(1, goal1_visited), (2, goal2_visited)]
            other_goals = [goals[0], goals[1]]
            direction = 3
        priority_node = queue_to_process.pop()      # 弹出队头元素作为当前节点
        node = priority_node[-1][0]                 # 获取当前节点的Key
        node_path = priority_node[-1][1]            # 获取出发节点到当前节点的路径 
        path_cost = priority_node[0] - (heuristic(graph, node, other_goals[0]) 
                                        + heuristic(graph, node, other_goals[1]))/2 # 目标i节点到当前节点的路径耗费的权值
        visited_set[node] = node_path               # 将当前节点加入已访问节点字典
        neighbors = sorted(graph.neighbors(node))   # 获取当前节点的所有邻节点，按字母顺序排序
        for x in neighbors:
            if not x in visited_set: # 判断该邻节点是否已经访问过     
                new_path = list(node_path)
                new_path.append(x)                                     # 目标i节点到该节点的路径           
                new_path_cost = graph.get_edge_data(node, x)['weight'] # 当前节点到该邻节点的权值
                # 该节点与另外两个目标的欧氏距离的平均值
                heuristic_value = (heuristic(graph, x, other_goals[0]) + heuristic(graph, x, other_goals[1]))/2 
                new_path_cost += path_cost + heuristic_value
                new_node = [new_path_cost, queue_to_process._len, (x, new_path) ]
                if not queue_to_process.find_by_id(x): # 判断该邻节点是否是已探索过的节点(即已加入队列中)
                    queue_to_process.append(new_node)            # 未探索过则加入队列
                    current_score_dictionary[x] = new_path_cost  # 添加相应权值
                else: # 如果队列中存在该邻节点但当前探索的路径权值更低，则进行更新 
                    update_node_if_needed(queue_to_process, current_score_dictionary, new_node)
                # 如果该邻节点已经在另两个访问方向的第一个被访问过，则说明找到了一条目标节点x到目标节点y的新路径
                if x in other_visited_sets[0][1]: 
                    current_path = list(node_path)
                    other_path = list(other_visited_sets[0][1][x])
                    found_path = concat_paths(current_path, other_path) # 对新路径进行拼接
                    found_path_cost = path_cost_calculation(graph, found_path)      # 计算新路径的权值
                    # 更新路径可能的最小权值
                    path_idx = direction + other_visited_sets[0][0]     # 对应的路径序号(x+y)       
                    best_possible_value = best_score_by_pair[path_idx]  # 该路径当前的最小权值
                    if best_possible_value and best_possible_value > found_path_cost:
                        # 更新最小权值，并将新路径加入到路径队列中
                        best_score_by_pair[path_idx] = found_path_cost  
                        found_paths_by_pair[path_idx].append([found_path_cost, found_path])
                # 如果该邻节点已经在另两个访问方向的第二个被访问过，则说明找到了一条目标节点x到目标节点y的新路径
                if x in other_visited_sets[1][1]: 
                    current_path = list(node_path)
                    other_path = list(other_visited_sets[1][1][x])
                    found_path = concat_paths(current_path, other_path) # 对新路径进行拼接
                    found_path_cost = path_cost_calculation(graph, found_path)      # 计算新路径的权值
                    # 更新路径可能的最小权值
                    path_idx = direction + other_visited_sets[1][0]     # 对应的路径序号(x+y)       
                    best_possible_value = best_score_by_pair[path_idx]  # 该路径当前的最小权值
                    if best_possible_value and best_possible_value > found_path_cost:
                        # 更新最小权值，并将新路径加入到路径队列中
                        best_score_by_pair[path_idx] = found_path_cost  
                        found_paths_by_pair[path_idx].append([found_path_cost, found_path])
    path1_2 = []
    path1_3 = []
    path2_3 = []
    # 获取目标x和目标y之间找到的所有路径中权值最小的路径
    if found_paths_by_pair[3]._len > 0:
        path1_2 = found_paths_by_pair[3].pop()[1]
    if found_paths_by_pair[4]._len > 0:
        path1_3 = found_paths_by_pair[4].pop()[1]
    if found_paths_by_pair[5]._len > 0:
        path2_3 = found_paths_by_pair[5].pop()[1]
    
    if path1_2 and path1_3 and path2_3: # 如果三个目标之间两两存在路径
        final_queue = PriorityQueue()                           # 创建一个队列用于路径排序
        pair1 = construct_tridirectional_path(path1_2, path1_3) # 拼接 1 -> 2 和 1 -> 3
        pair2 = construct_tridirectional_path(path1_3, path2_3)
        pair3 = construct_tridirectional_path(path1_2, path2_3)
        final_queue.append([path_cost_calculation(graph, pair1), pair1])    # 将连通三个目标的路径加入队列
        final_queue.append([path_cost_calculation(graph, pair2), pair2])
        final_queue.append([path_cost_calculation(graph, pair3), pair3])
        # 返回拼接的路径中权值最小的
        return final_queue.pop()[1]
    elif path1_2 and path1_3: # 如果只有目标1、2和目标1、3间存在路径，则返回拼接的路径即可             
        return construct_tridirectional_path(path1_2, path1_3)
    elif path1_2 and path2_3:
        return construct_tridirectional_path(path1_2, path2_3)
    elif path1_3 and path2_3:
        return construct_tridirectional_path(path1_3, path2_3)
    else: # 如果无法找到一条路径能够连通三个目标，则返回空列表
        return [start, goal]
 
