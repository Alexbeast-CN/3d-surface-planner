"""
This module provides functions for process and visualizing OBJ files using Open3D.
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

    try:
        with open(filename, 'r', encoding='utf-8') as file:
            for line in file:
                tokens = line.split()
                if not tokens:
                    continue

                prefix = tokens[0]
                # Read vertices
                if prefix == 'v':
                    p = np.array([float(x) for x in tokens[1:]],
                                 dtype=np.float32)
                    vertices.append(p)
                # Read facets
                elif prefix == 'f':
                    f = np.array([int(x) - 1 for x in tokens[1:]], dtype=int)
                    faces.append(f)
    except FileNotFoundError as e:
        print(f"File not found: {e}")

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


def get_lineset_from_faces(faces: np.ndarray) -> np.ndarray:
    """
    Get the lines from the faces.

    Args:
        faces (np.ndarray): The faces as a NumPy array.

    Returns:
        np.ndarray: The lines as a NumPy array.
    """
    lines_list = []
    for face in faces:
        for i, _ in enumerate(face):
            lines_list.append([face[i], face[(i + 1) % len(face)]])
    return np.array(lines_list)


def view_obj(filepath):
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
    mesh.paint_uniform_color([0.5, 0.5, 0.5])  # grey

    # Create a point cloud
    points = o3d.geometry.PointCloud()
    points.points = o3d.utility.Vector3dVector(vertices)

    lines = get_lineset_from_faces(faces)

    # Create a line set
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    # Set point color to black
    colors = np.array([[0, 0, 0] for _ in range(len(vertices))])
    points.colors = o3d.utility.Vector3dVector(colors)

    # Visualize
    # pylint: disable=E1101
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add objs to the visualizer
    vis.add_geometry(mesh)
    vis.add_geometry(line_set)
    vis.add_geometry(points)

    # Run the visualizerobj_utils
    vis.run()


def main():
    """
    Main function to load and visualize an OBJ file.
    """
    # Load the .obj file
    cur_file_path = os.path.dirname(os.path.realpath(__file__))
    file_name = 'dragon_quad.obj'
    obj_file_path = os.path.join(cur_file_path, 'data', file_name)

    # Visualize the object
    view_obj(obj_file_path)


if __name__ == '__main__':
    main()
