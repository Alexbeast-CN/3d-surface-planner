"""
This module provides functions for loading and visualizing OBJ files using Open3D.
"""

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
                f = np.array([int(x)-1 for x in tokens[1:]], dtype=int)
                faces.append(f)

    vertices = np.array(vertices)
    faces = np.array(faces)
    return vertices, faces

def view_obj(filepath):
    """
    View an OBJ file as a point cloud and wireframe.

    Args:
        filepath (str): The path to the OBJ file.
    """
    vertices, faces = load_obj(filepath)

    # Create a point cloud
    points = o3d.geometry.PointCloud()
    points.points = o3d.utility.Vector3dVector(vertices)

    lines_list = []
    for face in faces:
        for i, _ in enumerate(face):
            lines_list.append([face[i], face[(i+1)%len(face)]])

    lines = np.array(lines_list, dtype=int)

    # Create a line set
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    # Set point color to gray
    colors = np.array([[0.5, 0.5, 0.5] for _ in range(len(vertices))])
    points.colors = o3d.utility.Vector3dVector(colors)

    # Visualize
    # pylint: disable=E1101
    o3d.visualization.draw_geometries([line_set, points])

def main():
    """
    Main function to load and visualize an OBJ file.
    """
    # Load the .obj file
    cur_file_path = os.path.dirname(os.path.realpath(__file__))
    file_name = 'spoon_quad.obj'
    obj_file_path = os.path.join(cur_file_path, '../data', file_name)

    # Visualize the object
    view_obj(obj_file_path)

if __name__ == '__main__':
    main()
