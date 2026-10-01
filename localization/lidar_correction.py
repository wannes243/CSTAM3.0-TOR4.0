from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from sensors.measurements import PoseMeasurement


@dataclass(frozen=True)
class LidarCorrectionConfig:
    sigma_xy_m: float = 0.05
    sigma_yaw_rad: float = 0.04


class LidarPoseCorrection:
    """Hardware-independent insertion point for scan matching / SLAM.

    This class deliberately does not pretend to perform scan matching. A real
    matcher can produce a PoseMeasurement and pass it to the EKF without any
    changes to the Webots controller or viewer protocol.
    """

    def __init__(self, config: LidarCorrectionConfig | None = None) -> None:
        self.config = config or LidarCorrectionConfig()

    def make_measurement(self, timestamp: float, x_m: float, y_m: float, yaw_rad: float) -> PoseMeasurement:
        covariance = np.diag([
            self.config.sigma_xy_m ** 2,
            self.config.sigma_xy_m ** 2,
            self.config.sigma_yaw_rad ** 2,
        ])
        return PoseMeasurement(
            timestamp=timestamp,
            x_m=x_m,
            y_m=y_m,
            yaw_rad=yaw_rad,
            covariance=covariance,
            source="lidar_scan_matcher",
        )
