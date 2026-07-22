from typing import Dict, List, Any, Optional
from collections import defaultdict
from pathlib import Path
import math

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# -- Rosbag Utils

def parse_bag(bagfile_path: str, topics: Optional[List[str]] = None) -> Dict[str, List[Any]]:
    """Stores rosbag into a dictionary of topics, where each topic has a corresponding list of messages."""
    bag = defaultdict(list)
    
    bagpath = Path(bagfile_path)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        if topics == None:
            connections = [x for x in reader.connections]
        else:
            connections = [x for x in reader.connections if x.topic in topics]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            bag[connection.topic].append(msg)
            
    return bag

# -- Math Utils
 
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw) in radians.
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z