import rospy
import roslib.message
from rospy_message_converter import message_converter

class CommandBase(object):
    def __init__(self):
        pass

class TopicPub(CommandBase):
    def __init__(self):
        self._pubs = {}
        self._msgs = {}
    def __call__(self, args, feedback=None):
        name = args['name']
        mtype = args['type']
        msg = args['msg']
        if not name in self._pubs:
            self._pubs[name] = rospy.Publisher(name,
                                               roslib.message.get_message_class(mtype),
                                               queue_size=1,
                                               latch=True)
        if not name in self._msgs:
            self._msgs[name] = message_converter.convert_dictionary_to_ros_message(mtype, msg)
        self._pubs[name].publish(self._msgs[name])
