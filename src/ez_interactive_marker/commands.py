from abc import ABCMeta, abstractmethod
import importlib
import rospy
import roslib.message
from rospy_message_converter import message_converter

def convert_dictionary_to_ros_request(service_type, dictionary):
    service_class = roslib.message.get_service_class(service_type)
    request = service_class._request_class()
    request_fields = dict(message_converter._get_message_fields(request))

    for field_name, field_value in dictionary.items():
        if field_name in request_fields:
            field_type = request_fields[field_name]
            field_value = message_converter._convert_to_ros_type(field_type, field_value)
            setattr(request, field_name, field_value)
        else:
            error_message = 'ROS message type "{0}" has no field named "{1}"'\
                .format(message_type, field_name)
            raise ValueError(error_message)
    return request

class CommandBase(object):
    """ base class for specifying the command at the time of menu click. """
    __metaclass__ = ABCMeta
    def __init__(self):
        pass

    @abstractmethod
    def __call__(self, args, feedback=None):
        return None

class TopicPub(CommandBase):
    """ Publish topic """
    def __init__(self):
        self._pubs = {}
        self._msgs = {}
    def __call__(self, args, feedback=None):
        rospy.loginfo('topic pub: ' + str(args))
        name = args['name']
        mtype = args['type']
        msg = args['data']
        if not name in self._pubs:
            self._pubs[name] = rospy.Publisher(name,
                                               roslib.message.get_message_class(mtype),
                                               queue_size=1,
                                               latch=True)
        if not name in self._msgs:
            self._msgs[name] = message_converter.convert_dictionary_to_ros_message(mtype, msg)
        self._pubs[name].publish(self._msgs[name])

class ServiceCall(CommandBase):
    """ Call service """
    def __init__(self):
        self._srvs = {}
        self._reqs = {}
    def __call__(self, args, feedback=None):
        rospy.loginfo('service call: ' + str(args))
        name = args['name']
        mtype = args['type']
        req = args['data']
        if not name in self._srvs:
            self._srvs[name] = rospy.ServiceProxy(name,
                                                  roslib.message.get_service_class(mtype))
        if not name in self._reqs:
            self._reqs[name] = convert_dictionary_to_ros_request(mtype, req)
        self._srvs[name](self._reqs[name])

class PyFunction(CommandBase):
    """ Call python function """
    def __init__(self):
        self._mods = {}
    def __call__(self, args, feedback=None):
        rospy.loginfo('python function: ' + str(args))
        mod = args['module']
        func = args['func']
        fa = args['args']
        if not mod in self._mods:
            self._mods[mod] = importlib.import_module(mod)
        getattr(self._mods[mod], func)(**fa)
