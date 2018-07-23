#!/usr/bin/env python

from __future__ import with_statement # for python 2.5
__author__ = ''

import roslib; roslib.load_manifest('pr2_fridge')
import rospy
from itertools import izip
import openravepy
import time
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


args=None
parser = OptionParser(description='Shows the environment and the robot.', usage = '')
OpenRAVEGlobalArguments.addOptions(parser)
parser.add_option('--planner',action="store",type='string',dest='planner',default=None, help='the planner to use')
parser.add_option('--target', action="store",type='string',dest='target',default='1', help='')
(options, leftargs) = parser.parse_args(args=args)

env = OpenRAVEGlobalArguments.parseAndCreate(options,defaultviewer=True)


env.Load(options.scene)

raw_input()
