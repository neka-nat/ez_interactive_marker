import os.path
import numpy as np
import yaml
import roslib.message
import tf.transformations as trans
from rospy_message_converter import message_converter
_to_msg = message_converter.convert_dictionary_to_ros_message
_type_name = 'visualization_msgs/InteractiveMarker'

def _include(loader, node):
    if node.value[0] != '/':
        file_name = os.path.join(os.path.dirname(loader.name), node.value)
    else:
        file_name = node.value
    with file(file_name) as inputfile:
        return yaml.load(inputfile)

def _to_quat(loader, node):
    val = [float(v.value) for v in node.value]
    quat = trans.quaternion_from_euler(*val)
    return {'x': quat[0], 'y': quat[1], 'z': quat[2], 'w': quat[3]}

def _to_uint(loader, node):
    msgclass = roslib.message.get_message_class(node.value[0].value)
    return getattr(msgclass, node.value[1].value)

yaml.add_constructor("!include", _include)
yaml.add_constructor("!euler", _to_quat)
yaml.add_constructor("!enum", _to_uint)
yaml.add_constructor("!degrees", lambda loader, node: np.deg2rad(float(node.value)))

def load_from_string(s):
    config = yaml.load(s)
    for k, v in config.items():
        config[k]['interactive_marker'] = _to_msg(_type_name, v['interactive_marker'])
    return config

def load(file_name):
    with open(file_name) as f:
        return load_from_string(f.read())
