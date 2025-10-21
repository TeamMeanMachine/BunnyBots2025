import argparse
from collections import defaultdict

from gtsam import (
    NonlinearFactorGraph,
    Cal3DS2,
    Point3,
    Pose3,
    Values,
    GenericProjectionFactorCal3DS2,
    DoglegOptimizer,
    RangeFactor3D,
    BetweenFactorPose3,
)
from gtsam.noiseModel import Diagonal, Isotropic
from gtsam.symbol_shorthand import X, T, C, P
import logging
import matplotlib.pyplot as plt
import numpy as np
import time
import transformations as tf

from data_utils import CameraInfo
from helpers import in2m, m2in, get_tag_corner_offset_T
from nt_manager import NTManager


PRIMARY_TARGET_ID = 18
TAG_SIZE = in2m(6.5)
ROBOT_TAG_T = tf.compose_matrix(
    translate=[in2m(14.5 + 11.875), 0.0, in2m(11.75)], angles=[0.0, 0.0, np.pi]
)
PRIMARY_TAG_CORNERS = [
    ROBOT_TAG_T.dot(get_tag_corner_offset_T(corner_id, TAG_SIZE))
    for corner_id in range(4)
]


class Calibration:
    def __init__(self, server=None):
        self.nt_manager = NTManager(server)

    def _wait_for_camera_infos(self):
        logging.info("Waiting for camera infos...")
        while True:
            camera_infos = self.nt_manager.get_camera_infos()
            if camera_infos and all(
                info.hasCalibration() for info in camera_infos.values()
            ):
                break
            time.sleep(0.1)
        logging.info(f"Received camera infos for {len(camera_infos)} cameras!")
        return camera_infos

    def _wait_for_camera_measurements(self, num_cameras: int):
        COLLECT_TIME = 2.0  # seconds
        logging.info(f"Waiting for measurements for {COLLECT_TIME} seconds...")
        camera_measurements = []
        cameraIDs = set()
        tag_ids_by_camera = defaultdict(set)  # camera_id, set(tag_ids)
        s = time.time()
        while time.time() < s + COLLECT_TIME:
            _, _, vision = self.nt_manager.get_next()
            if vision is None:
                continue
            for measurement in vision:
                camera_measurements.append(measurement)
                cameraIDs.add(measurement.cameraID)
                tag_ids_by_camera[measurement.cameraID].add(measurement.targetID)
            time.sleep(0.1)
        tag_ids = set.intersection(*tag_ids_by_camera.values())
        logging.info(
            f"Received {len(camera_measurements)} measurements from {len(cameraIDs)} cameras!"
        )
        logging.info(f"Seen tag IDs by all cameras: {tag_ids}")
        return camera_measurements, tag_ids

    def _camera_info_to_gtsam_caP(self, camera_info: CameraInfo):
        return Cal3DS2(
            camera_info.M00,
            camera_info.M11,
            0.0,
            camera_info.M02,
            camera_info.M12,
            camera_info.d0,
            camera_info.d1,
            camera_info.d2,
            camera_info.d3,
        )

    def run(self):
        camera_infos = self._wait_for_camera_infos()
        num_cameras = len(camera_infos)
        camera_measurements, tag_ids = self._wait_for_camera_measurements(num_cameras)

        # Create factor graph & initial estimates
        graph = NonlinearFactorGraph()
        initial_estimate = Values()

        # Add camera poses (high variance -- we are solving for this)
        # Assume translation is better than rotation
        camera_pose_noise = Diagonal.Sigmas([0.1, 0.1, 0.1, 1e-2, 1e-2, 1e-2])
        for i, camera_info in camera_infos.items():
            # Rotate to OpenCV camera coordinate system
            pose = Pose3(
                camera_info.getTransformMatrix().dot(
                    tf.compose_matrix(angles=[-0.5 * np.pi, 0, -0.5 * np.pi])
                )
            )
            graph.addPriorPose3(X(i), pose, camera_pose_noise)
            initial_estimate.insert(X(i), pose)

        # Add primary tag corner landmarks (low variance)
        point_noise = Isotropic.Sigma(3, 1e-12)
        for corner_id, pose in enumerate(PRIMARY_TAG_CORNERS):
            graph.addPriorPoint3(
                P(100 * PRIMARY_TARGET_ID + corner_id),
                Point3(pose[0][3], pose[1][3], pose[2][3]),
                point_noise,
            )
            initial_estimate.insert(
                P(100 * PRIMARY_TARGET_ID + corner_id),
                Point3(pose[0][3], pose[1][3], pose[2][3]),
            )

        # Add the rest of the tag corners as landmarks (high variance)
        tag_prior_pose_noise = Isotropic.Sigma(6, 10.0)
        tag_corner_pose_constraint = Diagonal.Sigmas(
            [1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3]
        )
        range_constraint = Isotropic.Sigma(1, 1e-2)
        for tag_id in tag_ids:
            if tag_id == PRIMARY_TARGET_ID:
                continue

            # Center of tag. Initialize somewhere forwards facing back at robot.
            center_pose = Pose3(
                tf.compose_matrix(translate=[3.0, 0.0, 0.0], angles=[0.0, 0.0, np.pi])
            )
            graph.addPriorPose3(T(tag_id), center_pose, tag_prior_pose_noise)
            initial_estimate.insert(T(tag_id), center_pose)

            for corner_id in range(4):
                # Corner as Pose3
                corner_offset = Pose3(get_tag_corner_offset_T(corner_id, TAG_SIZE))
                corner_pose = center_pose.compose(corner_offset)
                corner_pose_key = C(100 * tag_id + corner_id)
                initial_estimate.insert(corner_pose_key, corner_pose)
                graph.add(
                    BetweenFactorPose3(
                        T(tag_id),
                        corner_pose_key,
                        corner_offset,
                        tag_corner_pose_constraint,
                    )
                )

                # Corner as Point3
                corner_point_key = P(100 * tag_id + corner_id)
                initial_estimate.insert(corner_point_key, corner_pose.translation())
                graph.add(
                    RangeFactor3D(
                        corner_pose_key, corner_point_key, 0.0, range_constraint
                    )
                )

        # Add projection factors
        pixel_noise = Isotropic.Sigma(2, 1e-1)
        for measurement in camera_measurements:
            if measurement.targetID not in tag_ids:
                continue
            graph.push_back(
                GenericProjectionFactorCal3DS2(
                    np.array([measurement.pixelX, measurement.pixelY]),
                    pixel_noise,
                    X(measurement.cameraID),
                    P(100 * measurement.targetID + measurement.targetCornerID),
                    self._camera_info_to_gtsam_caP(camera_infos[measurement.cameraID]),
                )
            )

        # Optimize
        # print(initial_estimate)
        # print(graph)
        optimizer = DoglegOptimizer(graph, initial_estimate)
        result = optimizer.optimize()

        for i in range(num_cameras):
            # Convert from OpenCV coordinate system
            _, _, rot, trans, _ = tf.decompose_matrix(
                result.atPose3(X(i))
                .matrix()
                .dot(
                    np.linalg.inv(
                        tf.compose_matrix(angles=[-0.5 * np.pi, 0, -0.5 * np.pi])
                    )
                )
            )
            logging.info(
                f"Camera {i} translation: new Translation3d(Units.inchesToMeters({m2in(trans[0])}), Units.inchesToMeters({m2in(trans[1])}), Units.inchesToMeters({m2in(trans[2])}))"
            )
            logging.info(
                f"Camera {i} rotation: new Rotation3d(Units.degreesToRadians({np.rad2deg(rot[0])}), Units.degreesToRadians({np.rad2deg(rot[1])}), Units.degreesToRadians({np.rad2deg(rot[2])}))"
            )

        # Plot
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")

        for tag_id in tag_ids:
            # Corner Point
            xs = []
            ys = []
            zs = []
            for corner_id in range(4):
                landmark = result.atPoint3(P(100 * tag_id + corner_id))
                xs.append(landmark[0])
                ys.append(landmark[1])
                zs.append(landmark[2])
            ax.scatter(xs, ys, zs, marker=".")

            if tag_id == PRIMARY_TARGET_ID:
                continue

            # Corner Pose
            xs = []
            ys = []
            zs = []
            for corner_id in range(4):
                landmark = result.atPose3(C(100 * tag_id + corner_id))
                xs.append(landmark.translation()[0])
                ys.append(landmark.translation()[1])
                zs.append(landmark.translation()[2])
            ax.scatter(xs, ys, zs, marker="s")

            # Tag Pose
            tag = result.atPose3(T(tag_id))
            ax.scatter(
                [tag.translation()[0]],
                [tag.translation()[1]],
                tag.translation()[2],
                marker="x",
            )

        xs = []
        ys = []
        zs = []
        for i in range(num_cameras):
            _, _, rot, trans, _ = tf.decompose_matrix(result.atPose3(X(i)).matrix())
            xs.append(trans[0])
            ys.append(trans[1])
            zs.append(trans[2])
        ax.scatter(xs, ys, zs, marker="s")

        ax.set_box_aspect([1.0, 1.0, 1.0])
        ax.axes.set_xlim3d(left=-1, right=1)
        ax.axes.set_ylim3d(bottom=-1, top=1)
        ax.axes.set_zlim3d(bottom=-1, top=1)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--local", action="store_true")
    args = parser.parse_args()
    Calibration(
        "localhost" if args.local else "10.6.4.2",
    ).run()
