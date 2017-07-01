from collections import defaultdict
import yaml
from rospy_message_converter import message_converter
_to_msg = message_converter.convert_dictionary_to_ros_message
_type_name = 'visualization_msgs/InteractiveMarker'

def load(file_name):
    config = yaml.load(open(file_name))
    for k, v in config.items():
        config[k]['interactive_marker'] = _to_msg(_type_name, v['interactive_marker'])
    return config
