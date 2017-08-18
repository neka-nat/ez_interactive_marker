import functools
import rospy
import geometry_msgs.msg as gm
import visualization_msgs.msg as vm

def process_feedback(feedback=None):
    pass

class SubscriberGenerator(object):
    def __init__(self, server):
        self._server = server

    def generate(self, name):
        node_name = rospy.get_name()
        rospy.Subscriber('/'.join([node_name, name, 'update_interactive_marker']),
                         vm.InteractiveMarker,
                         functools.partial(self._update_interactive_marker, name))
        rospy.Subscriber('/'.join([node_name, name, 'update_pose']), gm.Pose,
                         functools.partial(self._update_interactive_marker_pose, name))
        rospy.Subscriber('/'.join([node_name, name, 'add_control']),
                         vm.InteractiveMarkerControl,
                         functools.partial(self._add_interactive_marker_control, name))
        rospy.Subscriber('/'.join([node_name, name, 'remove_control']),
                         vm.InteractiveMarkerControl,
                         functools.partial(self._remove_interactive_marker_control, name))

    def _update_interactive_marker(self, name, int_marker):
        int_marker_org = self._server.get(name)
        name_org = int_marker_org.name
        int_marker_org = int_marker
        int_marker_org.name = name_org
        self._server.insert(int_marker_org, process_feedback)
        self._server.applyChanges()

    def _update_interactive_marker_pose(self, name, pose):
        self._server.setPose(name, pose)
        self._server.applyChanges()

    def _add_interactive_marker_control(self, name, control):
        int_marker = self._server.get(name)
        cntrl = [c for c in int_marker.controls if c.name == control.name]
        if len(cntrl) == 0:
            int_marker.controls.append(control)
        else:
            cntrl[0] = control
        self._server.insert(int_marker, process_feedback)
        self._server.applyChanges()

    def _remove_interactive_marker_control(self, name, control):
        int_marker = self._server.get(name)
        int_marker.controls = [c for c in int_marker.controls if c.name != control.name]
        self._server.insert(int_marker, process_feedback)
        self._server.applyChanges()
