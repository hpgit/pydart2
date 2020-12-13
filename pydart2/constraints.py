from __future__ import absolute_import
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi
from .bodynode import BodyNode


class BallJointConstraint(object):
    def __init__(self, body1, body2, jointPos):
        self.body1 = body1
        self.body2 = body2
        self.jointPos = jointPos

    def add_to_world(self, world):
        papi.addBallJointConstraint(world.id,
                                    self.body1.skid,
                                    self.body1.id,
                                    self.body2.skid,
                                    self.body2.id,
                                    self.jointPos)

class BallJointConstraintV2(object):
    def __init__(self, body, jointPos):
        self.body = body
        self.jointPos = jointPos
        self.id = papi.createBallJointConstraint(body.wid, body.skid, body.id, jointPos)
        self.added = False
    
    def add_to_world(self, world):
        if not self.added:
            self.added = True
            papi.addBallJointConstraint(self.id)
    
    def activate(self, _activate=True):
        papi.setBallJointConstraintActivate(self.id, _activate)

    def remove_from_world(self, world):
        if self.added:
            self.added = False
            papi.removeBallJointConstraint(self.id)

    def set_joint_pos(self, pos):
        papi.setBallJointConstraintJointPos(self.id, pos)


class NonHolonomicContactConstraint(object):
    def __init__(self, body1, offset):
        """

        :param body1:
        :type body1: BodyNode
        :param offset:
        """
        self.body1 = body1
        self.offset = offset
        self.id = papi.createNonHolonomicContactConstraint(body1.wid, body1.skid, body1.id, self.offset)
        self.added = False

    def add_to_world(self):
        if not self.added:
            papi.addNonHolonomicContactConstraint(self.id)
            self.added = True

    def activate(self, _activate=True):
        papi.setNonHolonomicContactConstraintActivate(self.id, _activate)

    def set_violation_angle_ignore_threshold(self, th):
        papi.setViolationAngleIgnoreThreshold(self.id, th)

    def set_length_for_violation_ignore(self, length):
        papi.setLengthForViolationIgnore(self.id, length)

    def set_joint_pos(self, pos):
        papi.setNonHolonomicContactConstraintJointPos(self.id, pos)

    def set_projected_vector(self, vec):
        papi.setNonHolonomicContactConstraintProjectedVector(self.id, vec)


class NonHolonomicContactConstraintV2(object):
    def __init__(self, body1, offset1, offset2):
        """

        :param body1:
        :type body1: BodyNode
        :param offset1:
        :param offset2:
        """
        self.body1 = body1
        self.offset1 = offset1
        self.offset2 = offset2
        self.id = papi.createNonHolonomicContactConstraintV2(body1.wid, body1.skid, body1.id, self.offset1, self.offset2)
        self.added = False

    def add_to_world(self):
        if not self.added:
            papi.addNonHolonomicContactConstraintV2(self.id)
            self.added = True

    def activate(self, _activate=True):
        papi.setNonHolonomicContactConstraintActivateV2(self.id, _activate)

    def set_violation_angle_ignore_threshold(self, th):
        papi.setViolationAngleIgnoreThresholdV2(self.id, th)

    def set_prev_body_pos(self, pos):
        papi.setNonHolonomicContactConstraintPrevBodyPosV2(self.id, pos)

    def set_prev_body_vec(self, vec):
        papi.setNonHolonomicContactConstraintPrevProjectedVectorV2(self.id, vec)
