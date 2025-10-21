import cv2
import matplotlib.pyplot as plt
from multiprocessing import Process, Manager, Queue
import numpy as np
import os
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
from stl import mesh
import transformations as tf

from helpers import in2m, besph2cart
from particle_filter import (
    ROBOT_HALF_LENGTH,
    ROBOT_HALF_WIDTH,
    TARGET_CENTER_X,
    TARGET_CENTER_Y,
    TARGET_CENTER_Z,
    TARGET_RADIUS,
    FIELD_LENGTH,
    FIELD_WIDTH,
    FOV_HORIZ,
    FOV_VERT,
)

FIELD_IMAGE = os.path.join(os.path.dirname(__file__), "./field.png")

# FIELD_MESH = mesh.Mesh.from_file(os.path.join(os.path.dirname(__file__), "../../field.stl"))
# ROBOT_MESH = mesh.Mesh.from_file(os.path.join(os.path.dirname(__file__), "../../robot.stl"))

ROBOT_Z_HEIGHT = 0.1
ROBOT_LINES = np.array(
    [
        # Edges
        [ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        [-ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        [-ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        [-ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        [-ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        [ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        [ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        [ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, ROBOT_Z_HEIGHT],
        # Arrow
        [-0.1, 0.1, ROBOT_Z_HEIGHT],
        [0.2, 0.0, ROBOT_Z_HEIGHT],
        [0.2, 0.0, ROBOT_Z_HEIGHT],
        [-0.1, -0.1, ROBOT_Z_HEIGHT],
        [-0.1, -0.1, ROBOT_Z_HEIGHT],
        [-0.1, 0.1, ROBOT_Z_HEIGHT],
    ]
)
ROBOT_END_EFFECTOR = [
    [in2m(60.0), in2m(2.5), in2m(36.0)],
    [in2m(45.0), in2m(2.5), in2m(36.0)],
]

FOV_X = 1.0
FOV_Y = FOV_X * np.tan(0.5 * FOV_HORIZ)
FOV_Z = FOV_X * np.tan(0.5 * FOV_VERT)
FOV_TOP_LEFT = [FOV_X, FOV_Y, FOV_Z]
FOV_TOP_RIGHT = [FOV_X, -FOV_Y, FOV_Z]
FOV_BOTTOM_LEFT = [FOV_X, FOV_Y, -FOV_Z]
FOV_BOTTOM_RIGHT = [FOV_X, -FOV_Y, -FOV_Z]
FOV_LINES = np.array(
    [
        [0, 0, 0],
        FOV_TOP_LEFT,
        [0, 0, 0],
        FOV_TOP_RIGHT,
        [0, 0, 0],
        FOV_BOTTOM_LEFT,
        [0, 0, 0],
        FOV_BOTTOM_RIGHT,
        FOV_TOP_LEFT,
        FOV_TOP_RIGHT,
        FOV_TOP_RIGHT,
        FOV_BOTTOM_RIGHT,
        FOV_BOTTOM_RIGHT,
        FOV_BOTTOM_LEFT,
        FOV_BOTTOM_LEFT,
        FOV_TOP_LEFT,
    ]
)


class Plotter3d:
    def __init__(self, num_particles, is_red=True):
        self.num_particles = num_particles
        self.is_red = is_red
        self.camera_infos = {}
        self.fovs = []

    def start(self):
        self.q = Queue(maxsize=5)
        self.targets_q = Queue()
        self.camera_infos_q = Queue()
        self.p = Process(target=self.run)
        self.p.start()
        return self.q, self.targets_q, self.camera_infos_q

    def join(self):
        return self.p.join()

    def run(self):
        app = pg.mkQApp("QuixPF")
        self.w = gl.GLViewWidget()
        self.w.show()
        self.w.orbit(210, 0)
        self.w.setCameraPosition(distance=20)

        self.w.opts["distance"] = 2000
        self.w.opts["fov"] = 1

        ax = gl.GLAxisItem()
        ax.setSize(1, 1, 1)
        self.w.addItem(ax)

        grid = gl.GLGridItem()
        grid.setSize(18, 10)
        grid.setSpacing(1, 1)
        self.w.addItem(grid)

        field_img = plt.imread(FIELD_IMAGE)
        scale = FIELD_LENGTH / field_img.shape[1]
        image_item = gl.GLImageItem(pg.makeRGBA(field_img, levels=(0, 1))[0])
        image_item.scale(scale, scale, scale)
        image_item.rotate(-90, 0, 0, 1)
        image_item.translate(0, FIELD_WIDTH, 0)
        image_item.setGLOptions("opaque")
        self.w.addItem(image_item)

        # points = FIELD_MESH.points.reshape(-1, 3)
        # faces = np.arange(points.shape[0]).reshape(-1, 3)
        # mesh_data = gl.MeshData(vertexes=points, faces=faces)
        # field_mesh = gl.GLMeshItem(
        #     meshdata=mesh_data,
        #     smooth=True,
        #     drawFaces=True,
        #     drawEdges=True,
        #     color=(0.5, 0.5, 0.5, 1),
        #     edgeColor=(0, 0, 0, 1)
        # )
        # field_mesh.translate(0, 0, -0.01)
        # self.w.addItem(field_mesh)

        self.retroreflective_targets = gl.GLScatterPlotItem(
            color=(0, 1, 0, 1), size=0.1, pxMode=False
        )
        self.retroreflective_targets.setGLOptions("opaque")
        self.w.addItem(self.retroreflective_targets)

        # points = ROBOT_MESH.points.reshape(-1, 3)
        # faces = np.arange(points.shape[0]).reshape(-1, 3)
        # mesh_data = gl.MeshData(vertexes=points, faces=faces)
        # self.robot_mesh = gl.GLMeshItem(
        #     meshdata=mesh_data,
        #     smooth=True,
        #     drawFaces=True,
        #     drawEdges=True,
        #     color=(0.5, 0.5, 0.5, 1),
        #     edgeColor=(0, 0, 0, 1)
        # )
        # self.robot_mesh.scale(1e-3, 1e-3, 1e-3)
        # self.w.addItem(self.robot_mesh)

        self.robot = gl.GLLinePlotItem(
            pos=ROBOT_LINES, width=5, color=(0, 0, 0, 1), mode="lines"
        )
        self.robot.setGLOptions("opaque")
        self.w.addItem(self.robot)

        self.robot_end_effector = gl.GLScatterPlotItem(
            pos=ROBOT_END_EFFECTOR, color=(1, 1, 1, 1), size=0.1, pxMode=False
        )
        self.robot_end_effector.setGLOptions("opaque")
        self.w.addItem(self.robot_end_effector)

        self.particles = gl.GLScatterPlotItem(size=2)
        self.w.addItem(self.particles)

        # By target ID
        self.rays = {}

        timer = QtCore.QTimer()
        timer.timeout.connect(self._update)
        timer.start(int(1000.0 / 60.0))

        pg.exec()

    def _update(self):
        # Update targets
        while not self.targets_q.empty():
            retroreflective_targets, apriltag_targets = self.targets_q.get()
            if retroreflective_targets:
                pos = []
                for t in retroreflective_targets:
                    pos.append((t.x, t.y, t.z))
                self.retroreflective_targets.setData(pos=pos)

            if apriltag_targets:
                pos = []
                for t in apriltag_targets.values():
                    target = gl.GLMeshItem(
                        meshdata=gl.MeshData(
                            vertexes=np.array(
                                [
                                    [0.0, 0.5, 0.5],
                                    [0.0, -0.5, 0.5],
                                    [0.0, 0.5, -0.5],
                                    [0.0, -0.5, -0.5],
                                ]
                            ),
                            faces=np.array([[0, 1, 2], [3, 2, 1]]),
                        ),
                        smooth=True,
                        color=(1, 0, 1, 1),
                    )
                    target.setTransform(pg.Transform3D(t.getPoseMatrix()))
                    target.scale(t.size, t.size, t.size)
                    self.w.addItem(target)

        # Fast fowrard to the latest element
        latest = None
        while not self.q.empty():
            latest = self.q.get()

        if latest is not None:
            vision, particles, best_estimate, has_vision = latest

            # Particles
            if particles is not None:
                NUM_TO_PLOT = min(5000, particles.shape[0])
                pos = np.zeros((NUM_TO_PLOT, 3))
                pos[:, 0] = particles[:NUM_TO_PLOT, 0, 3]
                pos[:, 1] = particles[:NUM_TO_PLOT, 1, 3]
                self.particles.setData(
                    pos=pos, color=(0, 1, 0, 1) if has_vision else (1, 0, 0, 1)
                )

            # Robot
            x, y, theta = best_estimate
            field_robot_T = pg.Transform3D()
            field_robot_T.translate(x, y, 0.0)
            field_robot_T.rotate(np.rad2deg(theta), 0, 0, 1)
            self.robot.setTransform(field_robot_T)
            self.robot_end_effector.setTransform(field_robot_T)

            # self.robot_mesh.setTransform(field_robot_T)
            # self.robot_mesh.scale(1e-3, 1e-3, 1e-3)
            # self.robot_mesh.rotate(90, 0, 0, 1, local=True)

            # Update cameras
            while not self.camera_infos_q.empty():
                self.camera_infos = self.camera_infos_q.get()

                # Add untransformed FOVs as necessary
                while len(self.fovs) < len(self.camera_infos):
                    fov = gl.GLLinePlotItem(
                        pos=FOV_LINES, width=1, color=(1, 1, 1, 1), mode="lines"
                    )
                    fov.setGLOptions("opaque")
                    self.w.addItem(fov)
                    self.fovs.append(fov)

                # Transform FOV viz
                for i, fov in enumerate(self.fovs):
                    camera_info = self.camera_infos.get(i, None)
                    if camera_info is None:
                        continue
                    field_camera_T = field_robot_T * pg.Transform3D(
                        camera_info.getTransformMatrix()
                    )
                    fov.setTransform(field_camera_T)

            # Measurement rays
            detected_ids = set()
            if vision:
                for i, v in enumerate(vision):
                    camera_info = self.camera_infos.get(v.cameraID, None)
                    if camera_info is None or not camera_info.hasCalibration():
                        continue

                    # Add if we haven't seen this cameraID + target ID + cornerID before
                    id = f"{v.cameraID}-{-1 - i if v.targetID == -1 else v.targetID}-{v.targetCornerID}"
                    isRetro = v.targetID == -1

                    if id not in self.rays:
                        self.rays[id] = gl.GLLinePlotItem(mode="lines")
                        self.rays[id].setGLOptions("opaque")
                        self.w.addItem(self.rays[id])

                    # Update ID
                    detected_ids.add(id)

                    ray_length = in2m(25 * 12)
                    self.rays[id].setTransform(
                        field_robot_T * pg.Transform3D(camera_info.getTransformMatrix())
                    )

                    # Convert pixels to ray sfor plotting
                    cameraMatrix = camera_info.getCameraMatrix()
                    distCoeffs = camera_info.getDistCoeffs()
                    if cameraMatrix is None or distCoeffs is None:
                        continue
                    undistorted = cv2.undistortPoints(
                        np.array([v.pixelX, v.pixelY]),
                        cameraMatrix,
                        distCoeffs,
                    )

                    x, y = undistorted[0][0]
                    z = 1.0  # Assume z = 1 (on the normalized plane)

                    self.rays[id].setData(
                        pos=np.array([[0, 0, 0], ray_length * np.array([z, -x, -y])]),
                        color=(0, 1, 0, 1)
                        if isRetro
                        else ((1, 0, 1, 1) if v.targetCornerID == -1 else (0, 1, 1, 1)),
                        width=4 if isRetro else 2,
                    )
                    self.rays[id].setGLOptions("additive")
            # Turn off all unseen IDs
            for id, item in self.rays.items():
                if id not in detected_ids:
                    item.setGLOptions("additive")
                    item.setData(color=(0, 0, 0, 0))


if __name__ == "__main__":
    plotter = Plotter3d(5000)
    plotter_queue = plotter.start()
