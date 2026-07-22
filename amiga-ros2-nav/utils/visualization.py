from typing import Dict, List, Any, Tuple
import argparse

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from pyproj import Transformer

import utils

# -- General Functions

def center_with_first(points: List):
    points = np.array(points)
    ref_point = points[0]
    return points - ref_point

def plot_start_end(ax: Axes, X: List, Y: List):
    ax.plot(X[0], Y[0], marker='*', color='yellow', markersize=10, label='Start')
    ax.plot(X[-1], Y[-1], marker='X', color='red', markersize=10, label='End')

# -- EKF

def parse_odom_msg(msg: List[Any]):
    """Parses odom message to be used properly with matplotlib"""
    pos = msg.pose.pose.position
    quat = msg.pose.pose.orientation
    _, _, yaw = utils.euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
    return (pos.x, pos.y), yaw

def rotate_verts(verts: List[Tuple[float, float]], yaw_offset_rad: float) -> List[Tuple[float, float]]:
    """For tuning the yaw offset."""
    cos_yaw = np.cos(yaw_offset_rad)
    sin_yaw = np.sin(yaw_offset_rad)
    return [
        (x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw)
        for x, y in verts
    ]

def plot_odometry_path(ax: Axes, msgs: List[Any], label: str, plot_every: int = 1):
    assert len(msgs) > 0, "No messages in topic to plot!"
    
    # -- Parse msgs
    verts = []
    angles = []
    for i in range(0, len(msgs), plot_every):
        pos, angle = parse_odom_msg(msgs[i])
        verts.append(pos)
        angles.append(angle)
    verts = center_with_first(verts)
    verts = rotate_verts(verts, 0.0)

    # -- Plot Path and Orientation
    X, Y = zip(*verts)
    U = np.cos(angles)
    V = np.sin(angles)
    
    ax.plot(X, Y, marker="o", label=label)
    # ax.quiver(X, Y, U, V)
    
    # -- Plot start and end points
    plot_start_end(ax, X, Y)
    
# -- GPS

def parse_coordinates(msgs: List[Any]):
    coords = []
    for msg in msgs:
        coords.append(
            (msg.longitude, msg.latitude, msg.altitude)
        )
    return coords

def determine_zone(coord: Tuple[float, float, float]):
    lon, lat, _ = coord
    utm_zone = int((lon + 180) / 6) + 1
    hemisphere = 'north' if lat >= 0 else 'south'
    return utm_zone, hemisphere

def to_utm(coords: List[Tuple[float, float, float]]):
    utm_zone, hemisphere = determine_zone(coords[0])
    
    # -- Define Transformation
    transformer = Transformer.from_crs(
        crs_from="EPSG:4326",
        crs_to=f"+proj=utm +zone={utm_zone} +{hemisphere} +datum=WGS84 +units=m +no_defs",
        always_xy=True
    )
    
    # -- Transform points to UTM
    utm_coords = []
    for lon, lat, alt in coords:
        easting, northing = transformer.transform(lon, lat)
        utm_coords.append((easting, northing, alt))
    
    return utm_coords


def plot_utm_path(ax: Axes, msgs: List[Any], label: str, plot_every: int = 1):
    assert len(msgs) > 0, "No messages in topic to plot!"
    # -- Convert to UTM
    coords = parse_coordinates(msgs)
    utm_coords = center_with_first(to_utm(coords))
    
    # -- Plot UTM coords
    points = [utm_coords[i] for i in range(0, len(utm_coords), plot_every)]
    X, Y, _ = zip(*points)
    ax.plot(X, Y, marker='o', label=label)
    
    # -- Plot start and end points
    plot_start_end(ax, X, Y)