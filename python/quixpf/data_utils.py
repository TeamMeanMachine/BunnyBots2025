from dataclasses import dataclass
import numpy as np
import transformations as tf
from typing import List
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Pose2d
from wpiutil import wpistruct


@wpistruct.make_wpistruct
@dataclass
class CameraInfo:
    transform: Transform3d
    hasCameraMatrix: wpistruct.int32
    M00: wpistruct.double
    M01: wpistruct.double
    M02: wpistruct.double
    M10: wpistruct.double
    M11: wpistruct.double
    M12: wpistruct.double
    M20: wpistruct.double
    M21: wpistruct.double
    M22: wpistruct.double
    hasDistCoeffs: wpistruct.int32
    d0: wpistruct.double
    d1: wpistruct.double
    d2: wpistruct.double
    d3: wpistruct.double
    d4: wpistruct.double
    d5: wpistruct.double
    d6: wpistruct.double
    d7: wpistruct.double

    def __getstate__(self):
        state = self.__dict__.copy()
        # Manually serialize Transform3d
        state["__X"] = state["transform"].translation().X()
        state["__Y"] = state["transform"].translation().Y()
        state["__Z"] = state["transform"].translation().Z()
        state["__rotX"] = state["transform"].rotation().X()
        state["__rotY"] = state["transform"].rotation().Y()
        state["__rotZ"] = state["transform"].rotation().Z()
        del state["transform"]
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        # Manually deserialize Transform3d
        self.transform = Transform3d(
            Translation3d(state["__X"], state["__Y"], state["__Z"]),
            Rotation3d(state["__rotX"], state["__rotY"], state["__rotZ"]),
        )

    def getTransformMatrix(self):
        return tf.compose_matrix(
            translate=[
                self.transform.translation().X(),
                self.transform.translation().Y(),
                self.transform.translation().Z(),
            ],
            angles=[
                self.transform.rotation().X(),
                self.transform.rotation().Y(),
                self.transform.rotation().Z(),
            ],
        )

    def hasCalibration(self):
        return self.hasCameraMatrix and self.hasDistCoeffs

    def getCameraMatrix(self):
        if not self.hasCameraMatrix:
            return None
        return np.array(
            [
                [self.M00, self.M01, self.M02],
                [self.M10, self.M11, self.M12],
                [self.M20, self.M21, self.M22],
            ]
        )

    def getDistCoeffs(self):
        if not self.hasDistCoeffs:
            return None
        return np.array(
            [self.d0, self.d1, self.d2, self.d3, self.d4, self.d5, self.d6, self.d7]
        )


@wpistruct.make_wpistruct
@dataclass
class Fiducial:
    type: wpistruct.int32
    id: wpistruct.int32
    pose: Pose3d
    size: wpistruct.double

    def __getstate__(self):
        state = self.__dict__.copy()
        # Manually serialize Pose3d
        state["__X"] = state["pose"].translation().X()
        state["__Y"] = state["pose"].translation().Y()
        state["__Z"] = state["pose"].translation().Z()
        state["__rotX"] = state["pose"].rotation().X()
        state["__rotY"] = state["pose"].rotation().Y()
        state["__rotZ"] = state["pose"].rotation().Z()
        del state["pose"]
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        # Manually deserialize Pose3d
        self.pose = Pose3d(
            Translation3d(state["__X"], state["__Y"], state["__Z"]),
            Rotation3d(state["__rotX"], state["__rotY"], state["__rotZ"]),
        )

    def getPoseMatrix(self):
        return tf.compose_matrix(
            translate=[
                self.pose.translation().X(),
                self.pose.translation().Y(),
                self.pose.translation().Z(),
            ],
            angles=[
                self.pose.rotation().X(),
                self.pose.rotation().Y(),
                self.pose.rotation().Z(),
            ],
        )


@dataclass
class Vision:
    cameraID: int
    targetID: int
    targetCornerID: int
    pixelX: float
    pixelY: float
    pixelSigma: float


@dataclass
class Odometry:
    x: float
    y: float
    theta: float


@dataclass
class Measurement:
    id: int
    odom: Odometry
    vision: List[Vision]


@wpistruct.make_wpistruct
@dataclass
class PoseEstimate:
    id: wpistruct.int32
    pose: Pose2d
    hasVision: wpistruct.int32
