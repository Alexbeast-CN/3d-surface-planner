"""
This module contains the implementation of the 3D surface planner application.
It provides functionality for loading and displaying geometries, path planning,
and picking points from the point cloud.
"""

import os
import numpy as np
import open3d as o3d
from open3d.visualization import gui
from open3d.visualization import rendering  # pylint: disable=E0611
import a_star
import obj_utils


# pylint: disable=R0902, R0903
class App:
    """
    This class represents the application for the 3D surface planner.
    It provides functionality for loading and displaying geometries,
    path planning, and picking points from the point cloud.
    """
    def __init__(self, file_path: str):
        """
        Initializes the App object.

        Parameters:
        - file_path (str): The path to the file containing the geometries.
        """

        # We will create a SceneWidget that fills the entire window, and then
        # a label in the lower left on top of the SceneWidget to display the
        # coordinate.
        app = gui.Application.instance
        self.window = app.create_window("3d surface planner", 1440, 1080)

        self.window.set_on_layout(self._on_layout)
        self.widget3d = gui.SceneWidget()
        self.window.add_child(self.widget3d)

        # Initialize the picked point
        self.picked_point_indices = []

        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)

        # Initialize the geometries
        self.points = o3d.geometry.PointCloud()
        self.mesh = o3d.geometry.TriangleMesh()
        self.lines = o3d.geometry.LineSet()
        self._load_obj(file_path)
        self._display_geometries()

        # For path planning
        self.graph = a_star.Graph()
        self._build_graph()

        self.kdtree = o3d.geometry.KDTreeFlann(self.points)

        self.widget3d.set_on_mouse(self._on_mouse_widget3d)

    # pylint: disable=W0613
    def _on_layout(self, layout_context):
        r = self.window.content_rect
        self.widget3d.frame = r

    def _load_obj(self, filename: str):
        vertices, faces = obj_utils.load_obj(filename)

        # create points instance
        self.points.points = o3d.utility.Vector3dVector(vertices)

        # create lines instance
        lines_list = obj_utils.get_lineset_from_faces(faces)
        self.lines.points = o3d.utility.Vector3dVector(vertices)
        self.lines.lines = o3d.utility.Vector2iVector(lines_list)

        # create mesh instance
        tri_faces = obj_utils.quad_to_tri(faces)
        self.mesh.triangles = o3d.utility.Vector3iVector(tri_faces)
        self.mesh.vertices = o3d.utility.Vector3dVector(vertices)

    def _display_geometries(self):
        # Set the colors
        self.points.paint_uniform_color([0.5, 0.5, 1])  # light blue
        self.lines.paint_uniform_color([0.5, 0.5, 1])  # light blue
        self.mesh.paint_uniform_color([1.0, 1.0, 1.0])  # white

        # Set the materials
        point_mat = rendering.MaterialRecord()
        point_mat.shader = "defaultLit"
        point_mat.point_size = 10

        line_mat = rendering.MaterialRecord()
        line_mat.shader = "unlitLine"
        line_mat.line_width = 4

        mesh_mat = rendering.MaterialRecord()
        # mesh_mat.shader = "defaultLitSSR"

        # Add the geometries to the scene
        self.widget3d.scene.add_geometry("mesh", self.mesh, mesh_mat)
        self.widget3d.scene.add_geometry("lines", self.lines, line_mat)
        self.widget3d.scene.add_geometry("points", self.points, point_mat)

        # Set the camera
        bounds = self.widget3d.scene.bounding_box
        center = bounds.get_center()
        self.widget3d.setup_camera(60, bounds, center)

    def _build_graph(self):
        # Build the graph
        for i, vertex in enumerate(self.points.points):
            self.graph.add_node(i, pos=vertex)

        for line in self.lines.lines:
            self.graph.add_edge(line[0], line[1], 1)

        self.graph.build_adj_list()

    def _on_mouse_widget3d(self, event):
        # We could override BUTTON_DOWN without a modifier, but that would
        # interfere with manipulating the scene.
        if event.type == gui.MouseEvent.Type.BUTTON_DOWN and event.is_modifier_down(
                gui.KeyModifier.CTRL):

            # remove_geometry will remove the geometry properties too.
            # pylint: disable=R0914
            def depth_callback(depth_image):
                # Coordinates are expressed in absolute coordinates of the
                # window, but to dereference the image correctly we need them
                # relative to the origin of the widget. Note that even if the
                # scene widget is the only thing in the window, if a menubar
                # exists it also takes up space in the window (except on macOS).
                x = event.x - self.widget3d.frame.x
                y = event.y - self.widget3d.frame.y
                # Note that np.asarray() reverses the axes.
                depth = np.asarray(depth_image)[y, x]

                if depth != 1.0:  # not in the background
                    point = self.widget3d.scene.camera.unproject(
                        x, y, depth, self.widget3d.frame.width,
                        self.widget3d.frame.height)

                    # Get the point cloud index
                    [_, idx, _] = self.kdtree.search_knn_vector_3d(point, 1)
                    self.picked_point_indices.append(idx[0])

                # get path
                start_index = None
                end_index = None
                if len(self.picked_point_indices) > 0:
                    # Get the start and end points
                    start_index = self.picked_point_indices[0]
                    if len(self.picked_point_indices) > 1:
                        end_index = self.picked_point_indices[1]

                    # Remove previous start and end points and the path
                    self.widget3d.scene.remove_geometry("start")
                    self.widget3d.scene.remove_geometry("end")
                    self.widget3d.scene.remove_geometry("path")

                    # Init path planning visulization
                    start_sphere = o3d.geometry.TriangleMesh.create_sphere(
                        radius=0.5)
                    start_sphere.paint_uniform_color([1, 0, 0])  # red

                    end_sphere = o3d.geometry.TriangleMesh.create_sphere(
                        radius=0.5)
                    end_sphere.paint_uniform_color([0, 0, 1])  # blue

                    path_line_set = o3d.geometry.LineSet()
                    path_line_set.paint_uniform_color([1, 0, 0])  # red
                    path_line_set.points = o3d.utility.Vector3dVector(
                        self.points.points)

                    sphere_mat = rendering.MaterialRecord()

                    path_mat = rendering.MaterialRecord()
                    path_mat.shader = "unlitLine"
                    path_mat.line_width = 10

                    # draw start point
                    start_point = self.points.points[start_index]
                    start_sphere.translate(
                        [start_point[0], start_point[1], start_point[2]])
                    self.widget3d.scene.add_geometry("start", start_sphere,
                                                     sphere_mat)

                    if end_index is not None:
                        self.picked_point_indices.pop(0)

                        # draw end point
                        end_point = self.points.points[end_index]
                        end_sphere.translate(
                            [end_point[0], end_point[1], end_point[2]])
                        self.widget3d.scene.add_geometry(
                            "end", end_sphere, sphere_mat)

                        # Find the path
                        path = self.graph.astar(start_index, end_index)
                        path_lines = [[path[i], path[i + 1]]
                                      for i in range(len(path) - 1)]

                        path_line_set.lines = o3d.utility.Vector2iVector(
                            path_lines)
                        self.widget3d.scene.add_geometry(
                            "path", path_line_set, path_mat)

            self.widget3d.scene.scene.render_to_depth_image(depth_callback)

            return gui.Widget.EventCallbackResult.HANDLED
        return gui.Widget.EventCallbackResult.IGNORED


def main():
    """
    Entry point of the application.
    """
    app = gui.Application.instance
    app.initialize()

    cur_file_path = os.path.dirname(os.path.realpath(__file__))
    file_name = 'plate_1_quad.obj'
    obj_file_path = os.path.join(cur_file_path, '../data', file_name)

    App(obj_file_path)

    app.run()


if __name__ == "__main__":
    main()
