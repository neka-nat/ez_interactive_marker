#!/usr/bin/env python
PKG = 'ez_interactive_marker'
import roslib; roslib.load_manifest(PKG)
import unittest
import tempfile
import numpy as np
import ez_interactive_marker.config_parser as ec 

class TestConfigParser(unittest.TestCase):
    def test_load_include(self):
        tpf = tempfile.NamedTemporaryFile()
        f = open(tpf.name, 'w')
        f.write('{"test": 1}')
        f.close()
        src = '{value: !include %s}' % tpf.name
        dst = ec.yaml.load(src)
        self.assertEqual(dst['value'], {'test': 1})

    def test_load_enum(self):
        src = '{value: !enum [visualization_msgs/Marker, CUBE]}'
        dst = ec.yaml.load(src)
        self.assertEqual(dst['value'], 1)

    def test_load_degrees(self):
        src = '{value: !degrees 180.0}'
        dst = ec.yaml.load(src)
        self.assertAlmostEqual(dst['value'], np.pi)

    def test_load_euler(self):
        src = '{value: !euler [0.0, 0.0, 0.0]}'
        dst = ec.yaml.load(src)
        self.assertAlmostEqual(dst['value']['x'], 0.0)
        self.assertAlmostEqual(dst['value']['y'], 0.0)
        self.assertAlmostEqual(dst['value']['z'], 0.0)
        self.assertAlmostEqual(dst['value']['w'], 1.0)

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'test_config_parser', TestConfigParser)
