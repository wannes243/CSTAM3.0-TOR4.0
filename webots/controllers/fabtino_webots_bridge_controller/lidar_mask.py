"""Static horizontal LiDAR mask, independent of Webots/ROS for testing."""
from collections.abc import Mapping, Sequence
import math
from numbers import Real
from pathlib import Path
import warnings

ANGLE_TOLERANCE_DEG = 1e-6
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[3] / 'config/robot_parameters.yaml'


def load_mask(lidar, path=DEFAULT_CONFIG_PATH):
    """Read once at startup (or an explicit reload); no file I/O per scan."""
    import yaml
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if config is None:
        config = {}
    if not isinstance(config, Mapping):
        raise ValueError('Robot configuration must be a mapping')
    section = config.get('lidar', {})
    if not isinstance(section, Mapping):
        raise ValueError('lidar must be a mapping')
    return AngularMask(lidar.getFov(), lidar.getHorizontalResolution(),
                       lidar.getNumberOfLayers(), section.get('masked_angle_ranges', []))


class AngularMask:
    def __init__(self, fov_rad, resolution, layers, ranges):
        self.fov_rad = float(fov_rad)
        self.resolution = resolution
        self.layers = layers
        if not math.isfinite(self.fov_rad) or not 0 < self.fov_rad <= 2*math.pi:
            raise ValueError('LiDAR FOV must be finite and within (0, 2*pi] radians')
        if any(isinstance(v, bool) or not isinstance(v, int) or v < 1
               for v in (resolution, layers)):
            raise ValueError('LiDAR resolution and layer count must be positive integers')
        if not isinstance(ranges, Sequence) or isinstance(ranges, (str, bytes)):
            raise ValueError('lidar.masked_angle_ranges must be a list of [start, end] pairs')
        half_fov = math.degrees(self.fov_rad)/2
        intervals = []
        for index, interval in enumerate(ranges):
            label = f'lidar.masked_angle_ranges[{index}]'
            if (not isinstance(interval, Sequence) or isinstance(interval, (str, bytes))
                    or len(interval) != 2):
                raise ValueError(f'{label} must contain exactly two numbers')
            if any(isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v) for v in interval):
                raise ValueError(f'{label} bounds must be finite numbers')
            start, end = map(float, interval)
            if start > end:
                raise ValueError(f'{label}: start must be <= end')
            if start < -half_fov or end > half_fov:
                raise ValueError(f'{label} must be within [{-half_fov:g}, {half_fov:g}] degrees')
            intervals.append((start, end))
        merged = []
        overlap = False
        for start, end in sorted(intervals):
            if merged and start <= merged[-1][1]:
                overlap = True
                merged[-1] = (merged[-1][0], max(end, merged[-1][1]))
            else:
                merged.append((start, end))
        if overlap:
            warnings.warn('Overlapping lidar.masked_angle_ranges merged into their union',
                          UserWarning, stacklevel=2)
        self.intervals = tuple(merged)
        step = 2*half_fov/(resolution-1) if resolution > 1 else 0.0
        # Requested local convention: 0 forward, positive CCW, first ray at
        # -FOV/2 (right edge). No robot yaw or map-frame transform belongs here.
        self.masked_indices = frozenset(
            i for i in range(resolution)
            if any(start-ANGLE_TOLERANCE_DEG <= -half_fov+i*step <= end+ANGLE_TOLERANCE_DEG
                   for start, end in merged))
        self.flat_indices = tuple(layer*resolution+i for layer in range(layers)
                                  for i in sorted(self.masked_indices))

    def filter_scan(self, range_image):
        out = list(range_image)  # Never mutate Webots' sensor buffer.
        if len(out) != self.resolution*self.layers:
            raise ValueError('LiDAR image shape changed; reload the mask before using this scan')
        # +inf means intentionally ignored/no return, never a point or obstacle.
        # Indices for every layer were precomputed, so no angle work per tick.
        for index in self.flat_indices:
            out[index] = math.inf
        return out
