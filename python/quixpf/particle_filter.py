import cv2
from getopt import getopt
from typing import Dict, List
from helpers import (
    cart2pol,
    pol2cart,
    in2m,
    cart2besph,
    cart2bepinhole,
    get_point_in_frame,
    get_x,
    set_x,
    get_y,
    set_y,
    set_yaw,
    get_tag_corner_offset_T,
)
import numpy as np
import transformations as tf

from data_utils import Odometry, Vision, CameraInfo

# SIR Particle Filter

ROBOT_HALF_LENGTH = 0.5 * in2m(36.0)  # m
ROBOT_HALF_WIDTH = 0.5 * in2m(36.0)  # m
MIN_ROBOT_DIMENSION = min(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH) # Used for handling field boundaries
FOV_HORIZ = np.deg2rad(70)
FOV_VERT = np.deg2rad(47.5)

FIELD_LENGTH = in2m(12 * 57 + 6.875)  # m
FIELD_WIDTH = in2m(12 * 26 + 5)  # m

TARGET_CENTER_X = 0.0  # m
TARGET_CENTER_Y = 0.0  # m
TARGET_CENTER_Z = 2.60  # m
TARGET_DIAMETER = 1.36  # m
TARGET_RADIUS = 0.5 * TARGET_DIAMETER  # m
# TARGET_CENTER_X = in2m(0)  # Book depository
# TARGET_CENTER_Y = in2m(0)
# TARGET_CENTER_Z = in2m(91)

# NUM_LANDMARKS = 16
# LANDMARK_ANGULAR_SPACING = 2.0 * np.pi / NUM_LANDMARKS
# LANDMARK_GAP_ANGULAR_OFFSET = np.deg2rad(24)
# LANDMARKS = []

# # Create landmarks CCW, with idx=0 being the first landmark from the +X axis.
# landmark_start_angle = LANDMARK_GAP_ANGULAR_OFFSET - 0.5 * LANDMARK_ANGULAR_SPACING
# for idx in range(NUM_LANDMARKS):
#     theta = landmark_start_angle * idx * LANDMARK_ANGULAR_SPACING
#     dx = GOAL_RADIUS * np.cos(theta)
#     dy = GOAL_RADIUS * np.sin(theta)
#     LANDMARKS.append(Landmark(
#         idx,
#         TARGET_CENTER_X + dx,
#         TARGET_CENTER_Y + dy,
#         TARGET_CENTER_Z,
#     ))

PARTICLE_LOG_LIKELIHOOD_ACCEPT_TOLERANCE = -300.0
FRACTION_OF_PARTICLES_OVER_TOLERANCE = 1e-4
REINITIALIZE_THRESHOLD = 10
INIT_ALLOWANCE = 30

# Helper function to compute point on target circle closest to the given x, y
def get_point_on_target_circle(x, y):
    norm = np.sqrt(x * x + y * y)
    scalar = TARGET_RADIUS / norm
    dx = x * scalar
    dy = y * scalar
    zs = np.full(dx.shape, TARGET_CENTER_Z)

    points = np.empty((x.shape[0] if x.shape else 1, 3, 1))
    points[:, 0, 0] = np.array([TARGET_CENTER_X + dx])
    points[:, 1, 0] = np.array([TARGET_CENTER_Y + dy])
    points[:, 2, 0] = zs
    return points


class ParticleFilter:
    def __init__(
        self,
        num_particles: int,
        priori: Odometry,
        retroreflective_targets,
        apriltag_targets,
    ):
        # Always start with same random seed for reproducibility
        np.random.seed(0)

        self.num_particles = num_particles
        # Keep track of last odometry to compute deltas.
        self.last_odom = priori
        self.retroreflective_targets = retroreflective_targets
        self.apriltag_targets = apriltag_targets
        self.initialize()

    def initialize(self, reinitialize=False):
        # Represent particles as an Nx4x4 matrix
        # N is the number of particles and each 4x4 matrix is a 3D transformation matrix
        self.particles = np.repeat(
            np.eye(4)[np.newaxis, :, :], self.num_particles, axis=0
        )
        # We need to keep track of the non-wrapping continuous yaw separately, since the 3D rotation matrix loses this info.
        self.particle_continuous_yaw = np.empty((self.num_particles, 1))
        self.particle_weights = np.full(
            self.num_particles, 1.0 / float(self.num_particles)
        )
        self.particle_indexes = np.array(
            range(self.num_particles)
        )  # Used for resampling

        if reinitialize:
            # Uniform distribution across the field
            xs = np.random.uniform(0, FIELD_LENGTH, self.num_particles)
            ys = np.random.uniform(0, FIELD_WIDTH, self.num_particles)
        else:
            # Priori sigmas
            sigmaX = 3.0 if reinitialize else 5e-2
            sigmaY = 3.0 if reinitialize else 5e-2
            xs = np.random.normal(self.last_odom.x, sigmaX, self.num_particles)
            ys = np.random.normal(self.last_odom.y, sigmaY, self.num_particles)

        sigmaTheta = 3e-2  # We don't fully trust our starting yaw
        self.particle_continuous_yaw = np.random.normal(
            self.last_odom.theta, sigmaTheta, self.num_particles
        )
        set_x(self.particles, xs)
        set_y(self.particles, ys)
        set_yaw(self.particles, self.particle_continuous_yaw)

        # Used to reinitialize
        self.consecutive_bad_measurements = 0
        self.n_since_init = 0

    def predict(self, odometry: Odometry):
        # Compute delta motion
        T = tf.compose_matrix(
            translate=[odometry.x, odometry.y, 0], angles=[0, 0, odometry.theta]
        )
        last_T = tf.compose_matrix(
            translate=[self.last_odom.x, self.last_odom.y, 0],
            angles=[0, 0, self.last_odom.theta],
        )
        self.last_odom = odometry

        M = np.linalg.inv(last_T).dot(T)
        _, _, angles, trans, _ = tf.decompose_matrix(M)
        dx, dy, _ = trans
        _, _, dtheta = angles

        # Precompute gaussian noise to apply to x, y, and theta for each particle.
        # Scale uncertianty by distance + a constant uncertainty
        xSigma = 1e-1 * np.abs(dx) + 1e-2
        ySigma = 1e-1 * np.abs(dy) + 1e-2
        xNoise = np.random.normal(0.0, xSigma, size=self.num_particles)
        yNoise = np.random.normal(0.0, ySigma, size=self.num_particles)

        # Rotational noise
        thetaSigma = 1e-6
        thetaNoise = np.random.normal(0.0, thetaSigma, size=self.num_particles)

        # Transform each particle by value + noise.
        transforms = np.repeat(np.eye(4)[np.newaxis, :, :], self.num_particles, axis=0)
        set_x(transforms, dx + xNoise)
        set_y(transforms, dy + yNoise)
        dTheta = dtheta + thetaNoise
        set_yaw(transforms, dTheta)

        self.particles = np.matmul(self.particles, transforms)
        self.particle_continuous_yaw += dTheta

        # Handle field boundaries
        set_x(
            self.particles,
            np.clip(
                get_x(self.particles),
                MIN_ROBOT_DIMENSION,
                FIELD_LENGTH - MIN_ROBOT_DIMENSION,
            ),
        )
        set_y(
            self.particles,
            np.clip(
                get_y(self.particles),
                MIN_ROBOT_DIMENSION,
                FIELD_WIDTH - MIN_ROBOT_DIMENSION,
            ),
        )

    def update(self, measurements: List[Vision], camera_infos: Dict[int, CameraInfo]):
        boundary_update = self.update_field_boundaries()
        vision_update = self.update_vision(measurements, camera_infos)
        if boundary_update or vision_update:
            self.resample()
        return vision_update

    def update_field_boundaries(self):
        xs = (get_x(self.particles),)
        ys = (get_y(self.particles),)

        within_positive_x = np.less(xs, FIELD_LENGTH)
        within_negative_x = np.greater(xs, 0.0)
        within_positive_y = np.less(ys, FIELD_WIDTH)
        within_negative_y = np.greater(ys, 0.0)

        in_bounds_x = np.logical_and(within_positive_x, within_negative_x)
        in_bounds_y = np.logical_and(within_positive_y, within_negative_y)
        in_bounds = np.logical_and(in_bounds_x, in_bounds_y)[0]

        if not np.all(in_bounds):
            # Update particle weights
            self.particle_weights *= in_bounds.astype(float)
            self.normalize_weights()
            return True
        return False

    def update_vision(
        self, measurements: List[Vision], camera_infos: Dict[int, CameraInfo]
    ):
        if measurements is None or len(measurements) < 1:
            return False

        # Compute and combine likelihoods for each target
        has_at_least_one_measurement = False
        log_likelihoods = np.zeros((self.num_particles,))
        for measurement in measurements:
            target = self.apriltag_targets.get(measurement.targetID, None)
            camera_info = camera_infos.get(measurement.cameraID, None)
            if camera_info is None or not camera_info.hasCalibration():
                continue
            if measurement.targetCornerID != -1:
                # AprilTag Corner
                T = target.getPoseMatrix().dot(
                    get_tag_corner_offset_T(measurement.targetCornerID, target.size)
                )
                log_likelihood = self.compute_single_measurement_log_likelihoods(
                    measurement,
                    T[0, 3],
                    T[1, 3],
                    T[2, 3],
                    camera_info,
                )
                has_at_least_one_measurement = True
            log_likelihoods += log_likelihood
        if not has_at_least_one_measurement:
            return False

        self.n_since_init += 1

        # Check if |FRACTION_OF_PARTICLES_OVER_TOLERANCE| have a log-likelihood greater than |PARTICLE_LOG_LIKELIHOOD_ACCEPT_TOLERANCE|
        over_tolerance = np.greater(
            log_likelihoods, PARTICLE_LOG_LIKELIHOOD_ACCEPT_TOLERANCE
        )
        if (np.sum(over_tolerance) > FRACTION_OF_PARTICLES_OVER_TOLERANCE * self.num_particles
        ):
            self.consecutive_bad_measurements = 0
            # Update particle weights
            self.particle_weights *= np.exp(log_likelihoods)
            self.normalize_weights()
            return True
        else:
            self.consecutive_bad_measurements += 1
            print(f"Bad measurement {self.consecutive_bad_measurements}/{REINITIALIZE_THRESHOLD}")

        if (
            self.consecutive_bad_measurements > REINITIALIZE_THRESHOLD
            and self.n_since_init > INIT_ALLOWANCE
        ):
            print("REINITIALIZING!!!!!!!!!!!!!!!!!!")
            self.initialize(reinitialize=True)

        return False

    def compute_single_measurement_log_likelihoods(self, measurement, x, y, z, camera_info: CameraInfo):
        assert camera_info.hasCalibration()

        # Compute the point of the target in the camera frame.
        camera_T = np.matmul(self.particles, camera_info.getTransformMatrix())
        target_point = np.empty((1, 3, 1))
        target_point[:, 0, 0] = x
        target_point[:, 1, 0] = y
        target_point[:, 2, 0] = z
        points = get_point_in_frame(camera_T, target_point)

        # Transformation to OpenCV's coordinate system
        points_opencv = np.zeros_like(points)
        points_opencv[:, 0] = -points[:, 1]  # Map -Y → X
        points_opencv[:, 1] = -points[:, 2]  # Map -Z → Y
        points_opencv[:, 2] = points[:, 0]  # Map X → Z

        # Compare the expected measurement with the actual measurement
        expected_measurement, _ = cv2.projectPoints(
            points_opencv,
            np.array([0.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0]),
            camera_info.getCameraMatrix(),
            camera_info.getDistCoeffs(),
        )
        expected_measurement = np.squeeze(expected_measurement)

        delta_measurement = expected_measurement - np.array(
            [measurement.pixelX, measurement.pixelY]
        )

        # Compute the likelihood of the measurement
        errors = np.linalg.norm(delta_measurement, axis=1)
        likelihoods = (
            1.0 / np.sqrt(2.0 * np.pi * measurement.pixelSigma * measurement.pixelSigma)
        ) * np.exp(
            -(errors * errors) / (2.0 * measurement.pixelSigma * measurement.pixelSigma)
        )

        # Set likelihoods of points behind the camera to zero.
        behind_cam = points_opencv[:, 2] < 0.0
        likelihoods[behind_cam[:, 0]] = 0.0

        # Use log likelhiod for numerical stability
        log_likelihoods = np.log(likelihoods)
        return log_likelihoods

    def normalize_weights(self):
        weight_sum = sum(self.particle_weights)
        if weight_sum == 0:
            self.initialize(reinitialize=True)
        else:
            self.particle_weights /= weight_sum

    def resample(self):
        chosen_indices = np.random.choice(
            self.particle_indexes, size=self.num_particles, p=self.particle_weights
        )
        self.particles = self.particles[chosen_indices, :, :]
        self.particle_continuous_yaw = self.particle_continuous_yaw[chosen_indices]
        self.particle_weights = np.full(
            self.num_particles, 1.0 / float(self.num_particles)
        )

    def get_best_estimate(self):
        weighted_x = np.sum(get_x(self.particles) * self.particle_weights)
        weighted_y = np.sum(get_y(self.particles) * self.particle_weights)
        weighted_yaw = np.sum(self.particle_continuous_yaw * self.particle_weights)
        return weighted_x, weighted_y, weighted_yaw

    # def get_visible_targets_from_pose(self, pose, camera_id):
    #     x, y, theta = pose
    #     camera_T = np.matmul(
    #         tf.compose_matrix(translate=[x, y, 0], angles=[0, 0, theta]),
    #         CAMERAS_TRANSFORMS[camera_id],
    #     )

    #     visible_targets = []
    #     for target in self.retroreflective_targets:
    #         target_point = np.array([[target.x], [target.y], [target.z]])
    #         point = get_point_in_frame(camera_T, target_point)
    #         expected_measurement = cart2bepinhole(point[0], point[1], point[2])
    #         if abs(expected_measurement[0]) < (0.5 * FOV_HORIZ) and abs(
    #             expected_measurement[1]
    #         ) < (0.5 * FOV_VERT):
    #             visible_targets.append(target)
    #     return visible_targets

    def plot_particles(self, plt, has_vision=False):
        plt.scatter(
            get_x(self.particles),
            get_y(self.particles),
            s=(self.particle_weights * 100),
            marker=".",
        )
        weighted_x = np.sum(get_x(self.particles) * self.particle_weights)
        weighted_y = np.sum(get_y(self.particles) * self.particle_weights)
        plt.scatter(
            weighted_x, weighted_y, marker="o", color="C2" if has_vision else "C3"
        )
        plt.plot()

    def plot_estimate(self, plt, has_vision=False, facing_our_goal=True):
        x, y, _ = self.get_best_estimate()
        plt.scatter(
            x,
            y,
            s=5,
            marker="o",
            color=("g" if facing_our_goal else "b") if has_vision else "r",
        )
        plt.plot()
