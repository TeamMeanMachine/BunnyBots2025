from collections import deque
import logging
import ntcore as nt
import numpy as np
from typing import Dict
from wpimath.geometry import Pose2d

from data_utils import Odometry, Vision, Fiducial, Measurement, CameraInfo, PoseEstimate

logger = logging.getLogger("NetworkTablesManager")
logging.basicConfig(level=logging.INFO)


class NTManager:
    def __init__(self, server=None):
        logger.info("Initializing NetworkTables...")
        self.inst = nt.NetworkTableInstance.getDefault()
        self.inst.setServer(server_name=server)
        self.inst.startClient4("quixsam")

        quixsam_table = self.inst.getTable("localizer")

        # Setup topics
        self.camera_infos_sub = quixsam_table.getStructArrayTopic(
            "cameras", CameraInfo
        ).subscribe([], nt.PubSubOptions(sendAll=True))
        self.targets_sub = quixsam_table.getStructArrayTopic(
            "targets", Fiducial
        ).subscribe([], nt.PubSubOptions(sendAll=True))
        self.measurements_sub = quixsam_table.getDoubleArrayTopic(
            "measurements"
        ).subscribe([], nt.PubSubOptions(sendAll=True))

        self.estimates_pub = quixsam_table.getStructTopic(
            "estimates", PoseEstimate
        ).publish(nt.PubSubOptions(sendAll=True))

        # Buffer to read measurements into
        self.last_measurement_id = -1  # Enforce IDs be always increasing
        self.measurement_buffer = deque()

        # Stores the transforms and calibration for each camera. Calibration parameters may start out undefined.
        self.camera_infos: Dict[int, CameraInfo] = {}

        # This should only be read once
        self.retroreflective_targets = []
        self.apriltag_targets = {}

        # Keep track of the last returned ID so we can rate limit what gets sent to the particle filter
        self.last_returned_id = 0
        self.last_vision_id = 0

    def is_connected(self):
        return self.inst.isConnected()

    def clear_table(self, table):
        for key in table.getKeys():
            table.delete(key)

    def measurement_received(self, value):
        STATIC_DATA_LENGTH = 8
        VISION_DATA_LENGTH = 5
        try:
            id_ = int(value[0])
            if id_ <= self.last_measurement_id:
                logging.critical(f"Measurement ID decreased")
            elif (
                all(np.isfinite(value))
                and (len(value) - STATIC_DATA_LENGTH) % VISION_DATA_LENGTH == 0
            ):
                vision = []
                for i in range(
                    int((len(value) - STATIC_DATA_LENGTH) / VISION_DATA_LENGTH)
                ):
                    vision.append(
                        Vision(
                            int(value[STATIC_DATA_LENGTH + VISION_DATA_LENGTH * i]),
                            int(value[STATIC_DATA_LENGTH + VISION_DATA_LENGTH * i + 1]),
                            int(value[STATIC_DATA_LENGTH + VISION_DATA_LENGTH * i + 2]),
                            value[STATIC_DATA_LENGTH + VISION_DATA_LENGTH * i + 3],
                            value[STATIC_DATA_LENGTH + VISION_DATA_LENGTH * i + 4],
                            value[7],
                        )
                    )
                self.measurement_buffer.append(
                    Measurement(
                        int(value[0]), Odometry(value[1], value[2], value[3]), vision
                    )
                )
                self.last_measurement_id = id_
            else:
                logging.critical(f"Invalid measurement data: {value}")
        except ValueError:
            logging.critical(f"Invalid measurement data")

    def get_camera_infos(self):
        for data in self.camera_infos_sub.readQueue():
            for i, camera_info in enumerate(data.value):
                self.camera_infos[i] = camera_info
        return self.camera_infos

    def get_targets(self):
        for data in self.targets_sub.readQueue():
            for target in data.value:
                if target.id == -1:
                    self.retroreflective_targets.append(target)
                else:
                    self.apriltag_targets[target.id] = target
        return self.retroreflective_targets, self.apriltag_targets

    def get_next(self):
        for data in self.measurements_sub.readQueue():
            self.measurement_received(data.value)

        logging.debug(f"Buffer size: {len(self.measurement_buffer)}")

        # Run buffer down to <= 5 measurements to avoid falling too far behind.
        drop_count = 0
        while len(self.measurement_buffer) > 5:
            self.measurement_buffer.popleft()
            drop_count += 1
        if drop_count > 0:
            logging.info(f"Dropped: {drop_count}")

        next_id = None
        next_odom = None
        next_vision = None
        if len(self.measurement_buffer) > 0:
            measurement = self.measurement_buffer.popleft()
            next_id = measurement.id
            next_odom = measurement.odom
            next_vision = measurement.vision if len(measurement.vision) > 0 else None

        # Exit early if there is no data
        if next_id is None:
            return None, None, None

        return next_id, next_odom, next_vision

    def publish_estimate(
        self, id_: int, x: float, y: float, theta: float, has_vision: bool
    ):
        self.estimates_pub.set(PoseEstimate(id_, Pose2d(x, y, theta), has_vision))
        self.inst.flush()
