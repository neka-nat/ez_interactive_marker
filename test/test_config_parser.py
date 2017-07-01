PKG = 'ez_interactive_marker'
import roslib; roslib.load_manifest(PKG)
import unittest
import numpy as np
import ez_interactive_marker.config_parser as ec 

class TestConfigParser(unittest.TestCase):
    def test_load_degree(self):
        src = '{value: !degree 180.0}'
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
