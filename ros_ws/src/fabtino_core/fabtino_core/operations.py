"""Validated viewer operations shared by the ROS adapters."""
import math
import numpy as np
from .navigation.planner import OccupancyGrid

MAX_MAP_CELLS = 1_000_000


def map_dimensions(payload):
    resolution = float(payload['resolution'])
    radius = float(payload['radius'])
    if not (math.isfinite(resolution) and math.isfinite(radius) and resolution > 0 and radius > 0):
        raise ValueError('Map resolution and radius must be finite and positive')
    size = math.ceil(2 * radius / resolution)
    if size * size > MAX_MAP_CELLS:
        raise ValueError('Map exceeds 1,000,000 cells; increase its resolution')
    return resolution,radius,size


def imported_grid(payload):
    resolution,radius,size=map_dimensions(payload)
    values = np.asarray(payload['grid'])
    if values.ndim != 1 or values.size != size * size or not np.isin(values, (-1, 0, 100)).all():
        raise ValueError('Map must contain exactly the expected number of -1/0/100 cells')
    grid = OccupancyGrid(resolution, radius)
    grid.load(resolution, radius, values)
    return grid
