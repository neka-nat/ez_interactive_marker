#!/usr/bin/env python
PKG = 'ez_interactive_marker'
import roslib; roslib.load_manifest(PKG)
import unittest
import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import ez_interactive_marker.controllers as ec 

class TestController(unittest.TestCase):
    def test_grouped_check_state(self):
        rospy.init_node("test")
        server = InteractiveMarkerServer("test")
        menu_handler = MenuHandler()
        controller = ec.GroupedCheckStateController(server, menu_handler)
        e0 = menu_handler.insert("menu0")
        controller.add_group("group0", e0)
        e1 = menu_handler.insert("menu1")
        controller.add_group("group0", e1)
        e2 = menu_handler.insert("menu2")
        controller.add_group("group0", e2)
        e3 = menu_handler.insert("menu3")
        controller.add_group("group1", e3)
        e4 = menu_handler.insert("menu4")
        controller.add_group("group1", e4)
        controller.update_check_state(e0)
        self.assertEqual(menu_handler.getCheckState(e0), MenuHandler.CHECKED)
        self.assertEqual(menu_handler.getCheckState(e1), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e2), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e3), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e4), MenuHandler.UNCHECKED)

        controller.update_check_state(e2)
        self.assertEqual(menu_handler.getCheckState(e0), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e1), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e2), MenuHandler.CHECKED)
        self.assertEqual(menu_handler.getCheckState(e3), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e4), MenuHandler.UNCHECKED)

        controller.update_check_state(e4)
        self.assertEqual(menu_handler.getCheckState(e0), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e1), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e2), MenuHandler.CHECKED)
        self.assertEqual(menu_handler.getCheckState(e3), MenuHandler.UNCHECKED)
        self.assertEqual(menu_handler.getCheckState(e4), MenuHandler.CHECKED)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_controller', TestController)
