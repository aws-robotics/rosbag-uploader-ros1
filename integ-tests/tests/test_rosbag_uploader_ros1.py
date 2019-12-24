#!/usr/bin/env python

import rostest
import unittest
import sys


PKG = 'rosbag_uploader_ros1_integration_tests'
NAME = 'rosbag_uploader_ros1_integration_tests'


class TestBareBones(unittest.TestCase):
    def test_dummy(self):
        self.assertEquals(1, 1, "1!=1")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestBareBones, sys.argv)
