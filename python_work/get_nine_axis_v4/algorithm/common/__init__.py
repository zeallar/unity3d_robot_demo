# -*- coding: utf-8 -*-
"""
These are the most common routines used to estimate attitude and heading.

"""

from algorithm.common import mathfuncs
from algorithm.common import orientation
from algorithm.common import geometry
from algorithm.common import frames

from algorithm.common.constants import M_PI, DEG2RAD, RAD2DEG
from algorithm.common.quaternion import Quaternion
from algorithm.common.dcm import DCM
