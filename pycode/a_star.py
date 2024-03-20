"""
This module provides functions using A* path planning on 3d surface
"""

import os
import heapq
import random

import open3d as o3d

from view_obj import load_obj, quad_to_tri

class Graph:
    """
    This is a class used to represent a Graph. The Graph is a data structure that 
    consists of a finite set of vertices(or nodes) and set of Edges which connect a pair of nodes.
    """
    def __init__(self):
        self.nodes = {}  # 节点字典，键为节点编号，值为节点的位置
        self.edges = {}  # 边字典，键为边的两个节点编号，值为边的权重
        self.adj_list = {}  # 邻接表，键为节点编号，值为与该节点相邻的节点列表

    def add_node(self, node_id, pos):
        """
        This method is used to add a node to the graph.

        Parameters:
        node_id (Any): The id of the node to be added.
        pos (Any): The position of the node to be added.
        """
        self.nodes[node_id] = pos

    def add_edge(self, u, v, weight):
        """
        This method is used to add an edge to the graph.

        Parameters:
        u (Any): The id of the first node of the edge.
        v (Any): The id of the second node of the edge.
        weight (Any): The weight of the edge.
        """
        self.edges[(u, v)] = weight

    def build_adj_list(self):
        """
        This method is used to build an adjacency list for the graph. 
        The adjacency list is a dictionary where the keys are node ids and 
        the values are lists of node ids that are connected by an edge.
        """
        for u, v in self.edges:
            if u not in self.adj_list:
                self.adj_list[u] = []
            if v not in self.adj_list:
                self.adj_list[v] = []
            self.adj_list[u].append(v)
            self.adj_list[v].append(u)

    def neighbors(self, current):
        """
        This method is used to get the neighbors of a given node in the graph.

        Parameters:
        current (Any): The id of the node whose neighbors are to be returned.

        Returns:
        list: A list of ids of the neighboring nodes.
        """
        return self.adj_list.get(current, [])

def build_graph(vertices, faces):
    """
    This function is used to build a graph from a list of vertices and faces.

    Parameters:
    vertices (list): A list of vertices, where each vertex is represented as a tuple of coordinates.
    faces (list): A list of faces, where each face is represented as a list of vertex indices.

    Returns:
    Graph: A Graph object representing the input vertices and faces.
    """
    graph = Graph()

    # 添加节点
    for i, vertex in enumerate(vertices):
        graph.add_node(i, pos=vertex)

    # Add edges
    for face in faces:
        for i, u in enumerate(face):
            v = face[(i + 1) % len(face)]
            graph.add_edge(u, v, weight=1)

    # 构建邻接表
    graph.build_adj_list()

    return graph

def heuristic(a, b, graph):
    """
    This function is used to calculate the heuristic value (estimated cost to reach the goal) 
    for the A* algorithm. Here, the Euclidean distance is used as the heuristic.

    Parameters:
    a (Any): The id of the current node.
    b (Any): The id of the goal node.
    graph (Graph): The graph on which the algorithm is being run.

    Returns:
    float: The heuristic value for the current node.
    """
    pos_a = graph.nodes[a]
    pos_b = graph.nodes[b]

    dx = pos_a[0] - pos_b[0]
    dy = pos_a[1] - pos_b[1]
    dz = pos_a[2] - pos_b[2]
    return (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5

def astar_path(graph, start, goal):
    """
    This function is used to find the shortest path from the start node
    to the goal node using the A* algorithm.

    Parameters:
    graph (Graph): The graph on which the algorithm is being run.
    start (Any): The id of the start node.
    goal (Any): The id of the goal node.

    Returns:
    list: The shortest path from the start node to the goal node as a list of node ids.
    """
    # 初始化优先队列和已访问集合
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    # 遍历图直到找到目标节点或队列为空
    while frontier:
        (_, current) = heapq.heappop(frontier)

        # 检查是否到达目标节点
        if current == goal:
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

def view_path(vertices, faces, path):
    """
    This function is used to visualize the path on the 3D mesh.

    Parameters:
    vertices (list): A list of vertices, where each vertex is represented as a tuple of coordinates.
    faces (list): A list of faces, where each face is represented as a list of vertex indices.
    path (list): The path to be visualized, represented as a list of vertex indices.

    """
    tri_faces = quad_to_tri(faces)

    # Create mesh
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                     o3d.utility.Vector3iVector(tri_faces))
    mesh.paint_uniform_color([1, 1, 1])  # white

    # Create a point cloud
    points = o3d.geometry.PointCloud()
    points.points = o3d.utility.Vector3dVector(vertices)

    lines_list = []
    for face in faces:
        for i, _ in enumerate(face):
            lines_list.append([face[i], face[(i + 1) % len(face)]])

    lines = o3d.geometry.LineSet()
    lines.points = o3d.utility.Vector3dVector(vertices)
    lines.lines = o3d.utility.Vector2iVector(lines_list)
    lines.paint_uniform_color([0.5, 0.5, 1])  # light blue

    # Create a line set for the path
    path_lines = []
    for i in range(len(path) - 1):
        path_lines.append([path[i], path[i + 1]])

    path_line_set = o3d.geometry.LineSet()
    path_line_set.points = o3d.utility.Vector3dVector(vertices)
    path_line_set.lines = o3d.utility.Vector2iVector(path_lines)
    path_line_set.paint_uniform_color([1, 0, 0])  # red

    # pylint: disable=E1101
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "unlitLine"
    mat.line_width = 10  # note that this is scaled with respect to pixels,

    # Visualize
    o3d.visualization.draw([
        {
            "name": "path_line_set",
            "geometry": path_line_set,
            "material": mat
        },
        {
            "name": "mesh",
            "geometry": mesh,
        },
        {
            "name": "lines",
            "geometry": lines,
        },
        {
            "name": "points",
            "geometry": points,
        }
    ])


def main():
    """
    This is the main function where the program starts. It initializes the graph, 
    runs the A* algorithm to find the shortest path, and then visualizes the path.
    """

    # Load the .obj file
    cur_file_path = os.path.dirname(os.path.realpath(__file__))
    file_name = 'plate_1_quad.obj'
    obj_file_path = os.path.join(cur_file_path, '../data', file_name)

    vertices, faces = load_obj(obj_file_path)

    graph = build_graph(vertices, faces)

    # 定义起始和目标节点（索引或位置）
    start_node_idx = random.randint(0, len(vertices) - 1)  # 随机选择起始点的索引
    goal_node_idx = random.randint(0, len(vertices) - 1)  # 随机选择目标点的索引

    print("随机生成的起始点索引：", start_node_idx)
    print("随机生成的目标点索引：", goal_node_idx)

    # 查找路径
    path = astar_path(graph, start_node_idx, goal_node_idx)
    print("Path:", path)
    
    view_path(vertices, faces, path)

if __name__ == "__main__":
    main()
