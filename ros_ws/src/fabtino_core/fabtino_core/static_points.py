"""Continuous world-frame evidence before a LiDAR return becomes static."""
from dataclasses import dataclass
import math


@dataclass
class Candidate:
    x: float
    y: float
    first_ns: int
    last_ns: int


class StaticPointFilter:
    def __init__(self, resolution, persistence_s=3., max_gap_s=.45, tolerance_m=None):
        self.resolution=float(resolution)
        if not all(math.isfinite(float(value)) for value in (resolution,persistence_s,max_gap_s)) or self.resolution<=0 or persistence_s<3 or max_gap_s<=0:
            raise ValueError('Static evidence requires finite positive dimensions and at least 3 seconds')
        self.persistence_ns=round(persistence_s*1e9)
        self.max_gap_ns=round(max_gap_s*1e9)
        self.tolerance=min(.02,self.resolution*.25) if tolerance_m is None else float(tolerance_m)
        if not math.isfinite(self.tolerance) or self.tolerance<=0:raise ValueError('Static position tolerance must be finite and positive')
        self.candidates={}
        self.last_ns=None

    def clear(self):
        self.candidates.clear();self.last_ns=None

    def forget(self,cells):
        for cell in cells:self.candidates.pop(cell,None)

    def observe(self,cell_points,timestamp_ns):
        """Return cells continuously observed at a fixed anchor for >3 seconds.

        Anchors never follow a moving return. Duplicate or out-of-order scans
        provide no evidence. A capture gap restarts the observation interval.
        """
        now=int(timestamp_ns)
        if self.last_ns is not None and now<=self.last_ns:return set()
        self.last_ns=now
        for cell in list(self.candidates):
            if now-self.candidates[cell].last_ns>self.max_gap_ns:del self.candidates[cell]
        accepted=set()
        for cell,points in cell_points.items():
            if not points:continue
            x=sum(point[0] for point in points)/len(points)
            y=sum(point[1] for point in points)/len(points)
            candidate=self.candidates.get(cell)
            if candidate is None or math.hypot(x-candidate.x,y-candidate.y)>self.tolerance:
                candidate=Candidate(x,y,now,now);self.candidates[cell]=candidate
            else:candidate.last_ns=now
            if now-candidate.first_ns>self.persistence_ns:accepted.add(cell)
        return accepted
