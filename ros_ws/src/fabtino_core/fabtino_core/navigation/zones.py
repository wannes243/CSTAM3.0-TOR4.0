"""Circular navigation exclusions in map coordinates, with robot clearance."""
from collections import deque
import math
import numpy as np


def finite(value,label):
    number=float(value)
    if not math.isfinite(number):raise ValueError(f'{label} must be finite')
    return number


def validate_config(payload):
    if not isinstance(payload,dict):raise ValueError('Navigation configuration must be an object')
    raw=payload.get('zones',[])
    if not isinstance(raw,list) or len(raw)>64:raise ValueError('Use at most 64 denied zones')
    zones=[];ids=set();names=set()
    for item in raw:
        if not isinstance(item,dict):raise ValueError('Invalid denied zone')
        if not isinstance(item.get('id'),str) or not isinstance(item.get('name'),str):raise ValueError('Zone identifiers and names must be text')
        zone_id=item['id'].strip();name=item['name'].strip()
        if not zone_id or len(zone_id)>128 or zone_id in ids:raise ValueError('Zone identifiers must be unique')
        if not name or len(name)>60 or name.casefold() in names:raise ValueError('Use a unique zone name of 1–60 characters')
        radius=finite(item['radius'],'Zone radius')
        if not .05<=radius<=100.:raise ValueError('Zone radius must be between 0.05 and 100 metres')
        zones.append(dict(id=zone_id,name=name,x=finite(item['x'],'Zone X'),y=finite(item['y'],'Zone Y'),radius=radius))
        ids.add(zone_id);names.add(name.casefold())
    base=payload.get('base')
    if base is not None:
        if not isinstance(base,dict):raise ValueError('Invalid base position')
        base=dict(x=finite(base['x'],'Base X'),y=finite(base['y'],'Base Y'),
                  yaw_deg=finite(base.get('yaw_deg',0.),'Base orientation'))
    return dict(zones=zones,base=base)


def point_denied(x,y,zones,clearance=0.):
    return next((zone for zone in zones if math.hypot(x-zone['x'],y-zone['y'])<=zone['radius']+clearance),None)


def segment_denied(a,b,zones,clearance=0.):
    dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
    for zone in zones:
        t=0. if length2==0 else max(0.,min(1.,((zone['x']-a[0])*dx+(zone['y']-a[1])*dy)/length2))
        if math.hypot(a[0]+t*dx-zone['x'],a[1]+t*dy-zone['y'])<=zone['radius']+clearance:return zone
    return None


def zone_mask(grid,zones,clearance=0.):
    """Block an entire cell intersecting the expanded circle, not just its centre."""
    blocked=np.zeros_like(grid.data,dtype=bool)
    coordinates=(np.arange(grid.size)+.5)*grid.resolution-grid.radius
    padding=grid.resolution/math.sqrt(2.)
    for zone in zones:
        radius=zone['radius']+clearance+padding
        xs=np.flatnonzero(abs(coordinates-zone['x'])<=radius)
        ys=np.flatnonzero(abs(coordinates-zone['y'])<=radius)
        if not len(xs) or not len(ys):continue
        region=(coordinates[ys,None]-zone['y'])**2+(coordinates[None,xs]-zone['x'])**2<=radius**2
        blocked[np.ix_(ys,xs)]|=region
    return blocked


def navigation_mask(grid,pose,zones,clearance):
    obstacles=grid.inflated_obstacles(clearance)
    blocked=obstacles|zone_mask(grid,zones,clearance)
    start=grid.cell(*pose[:2])
    # A safe continuous pose may occupy a cell whose far corner touches a zone.
    # Permit departure from that cell only; exact outgoing segments are checked.
    if start is not None and not obstacles[start[1],start[0]] and not point_denied(*pose[:2],zones,clearance):
        blocked[start[1],start[0]]=False
    return blocked


def approach_target(grid,pose,zones,zone_id,clearance=.6,buffer=.12,allow_unknown=True):
    """Choose the closest reachable cell outside the destination and other zones.

    Flood-fill once from the robot, preserving A*'s no-corner-cutting rule.
    Distance to the zone wins; travel distance breaks ties. This also handles
    an obstructed near side without selecting an unreachable boundary point.
    """
    zone=next((item for item in zones if item['id']==zone_id),None)
    if zone is None:raise ValueError('Delivery zone no longer exists')
    start=grid.cell(*pose[:2])
    if point_denied(*pose[:2],zones,clearance):raise ValueError('Robot has no safe start outside denied zones and obstacles')
    blocked=navigation_mask(grid,pose,zones,clearance)
    if start is None or not grid.traversable(start,blocked,allow_unknown):
        raise ValueError('Robot has no safe start outside denied zones and obstacles')
    reached=np.zeros_like(blocked,dtype=bool);reached[start[1],start[0]]=True
    queue=deque([(start,0)]);best=None
    minimum=zone['radius']+clearance+buffer
    while queue:
        (gx,gy),steps=queue.popleft();x,y=grid.point((gx,gy))
        distance=math.hypot(x-zone['x'],y-zone['y'])
        if distance>=minimum and (best is None or distance<best[0]-1e-6
                or (abs(distance-best[0])<=1e-6 and steps<best[1])):best=(distance,steps,x,y)
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nx,ny=gx+dx,gy+dy
            if not (0<=nx<grid.size and 0<=ny<grid.size) or reached[ny,nx] or not grid.traversable((nx,ny),blocked,allow_unknown):continue
            if (gx,gy)==start and segment_denied(pose[:2],grid.point((nx,ny)),zones,clearance):continue
            if dx and dy and (blocked[gy,nx] or blocked[ny,gx]):continue
            reached[ny,nx]=True;queue.append(((nx,ny),steps+1))
    if best is None:raise ValueError('No reachable delivery approach outside the denied zone')
    _,_,x,y=best
    return dict(x=x,y=y,yaw_deg=math.degrees(math.atan2(zone['y']-y,zone['x']-x)))


def motion_denied(pose,v,omega,zones,clearance=.6,horizon=.3):
    """Check the continuous swept command; pure rotation preserves circle clearance."""
    if abs(v)<1e-9:return point_denied(pose[0],pose[1],zones,clearance)
    previous=pose[:2]
    for index in range(1,13):
        t=horizon*index/12.
        if abs(omega)<1e-8:x,y=pose[0]+v*t*math.cos(pose[2]),pose[1]+v*t*math.sin(pose[2])
        else:
            x=pose[0]+v/omega*(math.sin(pose[2]+omega*t)-math.sin(pose[2]))
            y=pose[1]-v/omega*(math.cos(pose[2]+omega*t)-math.cos(pose[2]))
        # Bound the chord's approximation to the curved sweep.
        padding=abs(v*omega)*(horizon/12.)**2/8.
        zone=segment_denied(previous,(x,y),zones,clearance+padding)
        if zone:return zone
        previous=(x,y)
    return None
