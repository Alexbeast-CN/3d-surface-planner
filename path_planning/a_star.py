import numpy as np
import heapq

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import open3d as o3d
import random 
from typing import Tuple
import os
import numpy as np
import open3d as o3d




def load_obj(filename: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Load vertices and faces from an OBJ file.

    Args:
        filename (str): The path to the OBJ file.

    Returns:
        Tuple[np.ndarray, np.ndarray]: A tuple containing vertices and faces as NumPy arrays.
    """
    faces = []
    vertices = []

    with open(filename, 'r', encoding='utf-8') as file:
        for line in file:
            tokens = line.split()
            if not tokens:
                continue

            prefix = tokens[0]
            # Read vertices
            if prefix == 'v':
                p = np.array([float(x) for x in tokens[1:]], dtype=np.float32)
                vertices.append(p)
            # Read facets
            elif prefix == 'f':
                f = np.array([int(x) - 1 for x in tokens[1:]], dtype=int)
                faces.append(f)

    vertices = np.array(vertices)
    faces = np.array(faces)
    return vertices, faces

def quad_to_tri(faces: np.ndarray) -> np.ndarray:
    """
    Convert quad faces to triangle faces.

    Args:
        faces (np.ndarray): The faces as a NumPy array.

    Returns:
        np.ndarray: The triangle faces as a NumPy array.
    """
    tri_faces = []
    for face in faces:
        if len(face) == 4:
            tri_faces.append([face[0], face[1], face[2]])
            tri_faces.append([face[0], face[2], face[3]])
        else:
            tri_faces.append(face)
    return np.array(tri_faces)


def view_obj(filepath,path):
    """
    View an OBJ file as a point cloud and wireframe.

    Args:
        filepath (str): The path to the OBJ file.
    """
    vertices, faces = load_obj(filepath)
    tri_faces = quad_to_tri(faces)

    # Create mesh
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                     o3d.utility.Vector3iVector(tri_faces))
    mesh.paint_uniform_color([1, 1, 1])  # grey
    
    # Create a point cloud
    points = o3d.geometry.PointCloud()
    points.points = o3d.utility.Vector3dVector(vertices)

    lines_list = []
    for face in faces:
        for i, _ in enumerate(face):
            lines_list.append([face[i], face[(i + 1) % len(face)]])

    lines = np.array(lines_list, dtype=int)

    
    # Create a line set
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    # 设置线条的颜色为浅蓝色
    line_colors = np.array([[0.5, 0.5, 1] for _ in range(len(lines))])
    line_set.colors = o3d.utility.Vector3dVector(line_colors)

    
    # Set point color to black
    colors = np.array([[0.5, 0.5, 1] for _ in range(len(vertices))])
    points.colors = o3d.utility.Vector3dVector(colors)
    

    # Create a line set for the path
    path_lines = []
    for i in range(len(path) - 1):
        path_lines.append([path[i], path[i + 1]])  # 使用索引访问路径中的节点

    path_line_set = o3d.geometry.LineSet()
    path_line_set.points = o3d.utility.Vector3dVector(vertices)

   
    
    
    path_line_set.lines = o3d.utility.Vector2iVector(path_lines)
    path_colors = np.array([[1, 0, 0] for _ in range(len(path_lines))])
    path_line_set.colors = o3d.utility.Vector3dVector(path_colors)
    #path_line_set.line_width = 10  # 设置路径线宽度为10
  
    # Visualize
    # pylint: disable=E1101
    vis = o3d.visualization.Visualizer()
    vis.create_window()



    # Add objs to the visualizer
    vis.add_geometry(mesh)
    vis.add_geometry(line_set)
    vis.add_geometry(points)
    vis.add_geometry(path_line_set)

    
    

    # Run the visualizer
    vis.run()


class Graph:
    def __init__(self):
        self.nodes = {}  # 节点字典，键为节点编号，值为节点的位置
        self.edges = {}  # 边字典，键为边的两个节点编号，值为边的权重
        self.adj_list = {}  # 邻接表，键为节点编号，值为与该节点相邻的节点列表

    def add_node(self, node_id, pos):
        self.nodes[node_id] = pos

    def add_edge(self, u, v, weight):
        self.edges[(u, v)] = weight

    def build_adj_list(self):
        for (u, v) in self.edges.keys():
            if u not in self.adj_list:
                self.adj_list[u] = []
            if v not in self.adj_list:
                self.adj_list[v] = []
            self.adj_list[u].append(v)
            self.adj_list[v].append(u)

    def neighbors(self, current):
        return self.adj_list.get(current, [])

def build_graph(vertices, faces):
    graph = Graph()
    
    # 添加节点
    for i, vertex in enumerate(vertices):
        graph.add_node(i, pos=vertex)
    
    # 添加边
    for face in faces:
        for i in range(len(face)):
            u, v = face[i], face[(i + 1) % len(face)]
            graph.add_edge(u, v, weight=1)
    

    # 构建邻接表
    graph.build_adj_list()

    return graph

def heuristic(a, b, graph):
    # 启发式函数，这里使用欧氏距离作为估计
    pos_a = graph.nodes[a]
    pos_b = graph.nodes[b]
    return ((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2 + (pos_a[2] - pos_b[2]) ** 2) ** 0.5

def astar_path(graph, start, goal):
    # 初始化优先队列和已访问集合
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    # 遍历图直到找到目标节点或队列为空
    while frontier:
        (current_priority, current) = heapq.heappop(frontier)
        
        # 检查是否到达目标节点
        if current == goal:
            for next_node in graph.neighbors(current):
                print(next_node)
            break
        
        # 遍历当前节点的所有邻居
        for next_node in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.edges.get((current, next_node), float('inf'))
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(next_node, goal, graph)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current
                
    # 回溯找到完整路径
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    
    path.reverse()

    return path



# 示例使用
def main():

    # Load the .obj file
    cur_file_path = os.path.dirname(os.path.realpath(__file__))
    file_name = 'plate_1_quad.obj'
    obj_file_path = os.path.join(cur_file_path, '../data', file_name)

    vertices, faces = load_obj(obj_file_path)   

    #构建路径规划图
    #tri_faces = quad_to_tri(faces)
    graph = build_graph(vertices, faces)
    print(graph)

    

    # 随机选择起始点和目标点
    start_node = random.choice(vertices)
    goal_node = random.choice(vertices)

    print("随机生成的起始点：", start_node)
    print("随机生成的目标点：", goal_node)

    # 定义起始和目标节点（索引或位置）
    start_node_idx = random.randint(0, len(vertices)-1)  # 随机选择起始点的索引
    goal_node_idx = random.randint(0, len(vertices)-1)  # 随机选择目标点的索引

    print("随机生成的起始点索引：", start_node_idx)
    print("随机生成的目标点索引：", goal_node_idx)
    

    # 查找路径
    path = astar_path(graph, start_node_idx, goal_node_idx)
    print("Path:", path)

    # Visualize the object
    view_obj(obj_file_path,path)

    

if __name__ == "__main__":
    main()