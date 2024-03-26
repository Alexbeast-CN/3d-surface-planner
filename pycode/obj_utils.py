"""
This module provides functions for process and visualizing OBJ files using Open3D.
"""

from typing import Tuple
import os
import numpy as np
import open3d as o3d


def load_obj(filename: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Load vertices and faces from an OBJ file.

    Args:
        filename (str): The path to the OBJ file.

    Returns:
        Tuple[np.ndarray, np.ndarray, np.ndarray]: A tuple containing vertices, vertex_normals,
        and faces as NumPy arrays.
    """
    vertices = []
    v_normals = []
    faces = []

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
                # Read vertex normals
                elif prefix == 'vn':
                    n = np.array([float(x) for x in tokens[1:]],
                                 dtype=np.float32)
                    v_normals.append(n)
                # Read facets
                elif prefix == 'f':
                    f = np.array([int(x) - 1 for x in tokens[1:]], dtype=int)
                    faces.append(f)
    except FileNotFoundError as e:
        print(f"File not found: {e}")

    vertices = np.array(vertices)
    v_normals = np.array(v_normals)
    faces = np.array(faces)

    return vertices, v_normals, faces


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


def get_lineset_from_faces(
    faces: np.ndarray, external_edge_points=np.array([])) -> np.ndarray:
    """
    Get the lines from the faces.

    Args:
        faces (np.ndarray): The faces as a NumPy array.
        external_edge_points (np.ndarray): The external edge points as a NumPy array.

    Returns:
        np.ndarray: The lines as a NumPy array.
    """
    lines = []
    for face in faces:
        for i, _ in enumerate(face):
            v1 = face[i]
            v2 = face[(i + 1) % len(face)]
            if v1 not in external_edge_points \
                    and v2 not in external_edge_points \
                    and [v1, v2] not in lines \
                    and [v2, v1] not in lines:
                lines.append([v1, v2])
    return np.array(lines)


def cal_face_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """
    Calculate face normals.
    For triangle faces mesh, please use compute_triangle_normals() from open3d.

    Args:
        vertices (np.ndarray): The vertices as a NumPy array.
        faces (np.ndarray): The faces as a NumPy array.

    Returns:
        np.ndarray: The face normals as a NumPy array.
    """
    face_normals = []
    for face in faces:
        p0 = vertices[face[0]]
        p1 = vertices[face[1]]
        p2 = vertices[face[2]]
        v1 = p1 - p0
        v2 = p2 - p0
        normal = np.cross(v1, v2)
        # normalized face normal
        normal = normal / np.linalg.norm(normal)
        face_normals.append(normal)
    return np.array(face_normals)


def find_adj_faces(faces: np.ndarray):
    """
    Find adjacent faces for each vertex.

    Args:
        faces (np.ndarray): The faces as a NumPy array.
    """
    adj_faces = {}
    for i, face in enumerate(faces):
        adj_faces[i] = []
        for j, other_face in enumerate(faces):
            if i == j:
                continue
            common_vertices = set(face) & set(other_face)
            if len(common_vertices) == 2:
                adj_faces[i].append(j)

    return adj_faces


def find_external_edge_points(faces: np.ndarray, vertices: np.ndarray,
                              normals: np.ndarray):
    """
    Find external edge points.

    Args:
        faces (np.ndarray): The faces as a NumPy array.
        vertices (np.ndarray): The vertices as a NumPy array.
        normals (np.ndarray): The normals as a NumPy array.
    """
    edge_point = []
    for face in faces:
        for i, _ in enumerate(face):
            i1 = face[i]
            i2 = face[(i + 1) % len(face)]
            # Calculate the direction vector from p1 to p2
            direction = vertices[i1] - vertices[i2]
            # Normalize
            direction /= np.linalg.norm(direction)

            v2 = normals[i2]
            x_component = np.dot(v2, direction)
            y_component = np.linalg.norm(np.cross(direction, v2))
            # Calculate the angle of the second vector in the 2D plane
            angle = np.arctan2(y_component, x_component)

            if angle > 3 * np.pi / 5 - 1e-3:
                edge_point.append(i2)
                continue
    return np.array(edge_point)


def view_obj(obj_file_path):
    """
    View an OBJ file as a point cloud and wireframe.

    Args:

    """
    vertices, v_normals, faces = load_obj(obj_file_path)

    # Create mesh
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(quad_to_tri(faces))
    mesh.paint_uniform_color([0.3, 0.3, 0.3])  # grey

    if v_normals.size == 0:
        mesh.compute_vertex_normals()
        v_normals = mesh.vertex_normals

    # Create point cloud
    points = o3d.geometry.PointCloud()
    points.points = o3d.utility.Vector3dVector(vertices)
    points.paint_uniform_color([0.5, 0.5, 1])  # light blue

    edge_point = find_external_edge_points(faces, vertices, v_normals)
    lines = get_lineset_from_faces(faces, edge_point)

    # Create line set
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color([0.5, 0.5, 1])  # light blue

    # visualize vertices normal
    normals = o3d.geometry.LineSet()
    for i, v in enumerate(points.points):
        p1 = v
        p2 = v + v_normals[i] * 1
        normals.points.append(p1)
        normals.points.append(p2)
        normals.lines.append([2 * i, 2 * i + 1])

    normals.paint_uniform_color([1, 0, 0])  # red

    # Visualize
    # pylint: disable=E1101
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Add objs to the visualizer
    vis.add_geometry(mesh)
    vis.add_geometry(line_set)
    vis.add_geometry(points)
    vis.add_geometry(normals)

    # Run the visualizer
    vis.run()


def main():
    """
    Main function to load and visualize an OBJ file.
    """
    # Load the .obj file
    cur_file_path = os.path.dirname(os.path.realpath(__file__))
    file_name = 'plate_1_quad.obj'
    obj_file_path = os.path.join(cur_file_path, '../data', file_name)

    # Visualize the object
    view_obj(obj_file_path)


if __name__ == '__main__':
    main()
