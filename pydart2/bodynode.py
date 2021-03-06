from __future__ import absolute_import
# from builtins import range
# from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi
import numpy as np
from .shapenode import ShapeNode


class BodyNode(object):
    """
    :type skeleton: Skeleton
    :type _id: int
    :type name: str
    :type parent_bodynode: BodyNode
    :type child_bodynodes: list[BodyNode]
    :type parent_joint: Joint
    :type child_joints: list[Joint]
    :type dependent_dofs: list[Dof]
    :type markers: list[Marker]
    :type shapenodes: list[ShapeNode]
    """
    def __init__(self, _skel, _id):
        self.skeleton = _skel
        self._id = _id
        self.name = papi.bodynode__getName(self.wid, self.skid, self.id)
        self.parent_bodynode = None
        self.child_bodynodes = list()

        self.parent_joint = None
        self.child_joints = list()

        self.dependent_dofs = list()
        self.markers = list()  # Built by markers

        self.shapenodes = list()

    def build(self):
        # Build Body Nodes
        self.parent_bodynode = None
        self.child_bodynodes = list()

        ret_id = papi.bodynode__getParentBodyNode(self.wid, self.skid, self.id)
        if ret_id >= 0:
            self.parent_bodynode = self.skel.bodynodes[ret_id]

        num = papi.bodynode__getNumChildBodyNodes(self.wid, self.skid, self.id)
        for index in range(num):
            ret_id = papi.bodynode__getChildBodyNode(self.wid,
                                                     self.skid,
                                                     self.id,
                                                     index)
            if ret_id >= 0:
                self.child_bodynodes.append(self.skel.bodynodes[ret_id])

        # Build Joints
        self.parent_joint = None
        self.child_joints = list()
        ret_id = papi.bodynode__getParentJoint(self.wid, self.skid, self.id)
        if ret_id >= 0:
            self.parent_joint = self.skel.joints[ret_id]

        num = papi.bodynode__getNumChildJoints(self.wid, self.skid, self.id)
        for index in range(num):
            ret_id = papi.bodynode__getChildJoint(self.wid,
                                                  self.skid,
                                                  self.id,
                                                  index)
            if ret_id >= 0:
                self.child_joints.append(self.skel.joints[ret_id])

        # Build dependent_dofs
        self.dependent_dofs = list()
        num = papi.bodynode__getNumDependentDofs(self.wid, self.skid, self.id)
        for index in range(num):
            ret_id = papi.bodynode__getDependentDof(self.wid,
                                                    self.skid,
                                                    self.id,
                                                    index)
            if ret_id >= 0:
                self.dependent_dofs.append(self.skel.dofs[ret_id])

        # Build shapenodes
        self.shapenodes = list()
        num = papi.bodynode__getNumShapeNodes(self.wid, self.skid, self.id)
        for index in range(num):
            self.shapenodes.append(ShapeNode(self, index))

        # Build markers (by other class)
        self.markers = list()

    def num_child_bodynodes(self, ):
        return len(self.child_bodynodes)

    def num_dependent_dofs(self, ):
        return len(self.dependent_dofs)

    def num_child_joints(self, ):
        return len(self.child_joints)

    def num_markers(self, ):
        return len(self.markers)

    def num_shapenodes(self, ):
        return len(self.shapenodes)

    @property
    def id(self):
        return self._id

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def skel(self):
        return self.skeleton

    @property
    def skid(self):
        return self.skel.id

    # def num_contacts(self):
    #     return papi.getBodyNodeNumContacts(self.wid, self.skid, self.id)

    # def contacts(self):
    #     n = self.num_contacts()
    #     contacts = papi.getBodyNodeContacts(self.wid,
    #                                         self.skid,
    #                                         self.id, 7 * n)
    #     return [Contact(contacts[7 * i: 7 * (i + 1)]) for i in range(n)]

########################################
# Index Functions
    def index_in_skeleton(self, ):
        return papi.bodynode__getIndexInSkeleton(self.wid, self.skid, self.id)

    def index_in_tree(self, ):
        return papi.bodynode__getIndexInTree(self.wid, self.skid, self.id)

    def tree_index(self, ):
        return papi.bodynode__getTreeIndex(self.wid, self.skid, self.id)

########################################
# Property Functions
    def set_gravity_mode(self, _gravityMode):
        papi.bodynode__setGravityMode(self.wid,
                                      self.skid,
                                      self.id,
                                      _gravityMode)

    def gravity_mode(self, ):
        return papi.bodynode__getGravityMode(self.wid, self.skid, self.id)

    def is_collidable(self, ):
        return papi.bodynode__isCollidable(self.wid, self.skid, self.id)

    def set_collidable(self, _isCollidable):
        papi.bodynode__setCollidable(self.wid,
                                     self.skid,
                                     self.id,
                                     _isCollidable)

########################################
# Inertia Functions
    def mass(self):
        return papi.bodynode__getMass(self.wid, self.skid, self.id)

    @property
    def m(self):
        return self.mass()

    def set_mass(self, mass):
        return papi.bodynode__setMass(self.wid, self.skid, self.id, mass)

    def inertia(self):
        return papi.bodynode__getInertia(self.wid, self.skid, self.id)

    @property
    def I(self):
        return self.inertia()

    def set_inertia(self, I):
        return papi.bodynode__setInertia(self.wid, self.skid, self.id, I)

    def set_inertia_entries(self, Ixx, Iyy, Izz, Ixy=0.0, Ixz=0.0, Iyz=0.0):
        I = np.array([[Ixx, Ixy, Ixz],
                      [Ixy, Iyy, Iyz],
                      [Ixz, Iyz, Izz]])
        return self.set_inertia(I)

########################################
# Momentum Functions
    def local_com(self, ):
        return papi.bodynode__getLocalCOM(self.wid, self.skid, self.id)

    def com(self, ):
        return papi.bodynode__getCOM(self.wid, self.skid, self.id)

    @property
    def C(self):
        return self.com()

    def com_linear_velocity(self, ):
        return papi.bodynode__getCOMLinearVelocity(self.wid,
                                                   self.skid,
                                                   self.id)

    @property
    def dC(self):
        return self.com_linear_velocity()

    def com_spatial_velocity(self, ):
        return papi.bodynode__getCOMSpatialVelocity(self.wid,
                                                    self.skid,
                                                    self.id)

    def com_linear_acceleration(self, ):
        return papi.bodynode__getCOMLinearAcceleration(self.wid,
                                                       self.skid,
                                                       self.id)

    def com_spatial_acceleration(self, ):
        return papi.bodynode__getCOMSpatialAcceleration(self.wid,
                                                        self.skid,
                                                        self.id)

########################################
# Friction and Restitution Functions
    def set_friction_coeff(self, _coeff):
        papi.bodynode__setFrictionCoeff(self.wid, self.skid, self.id, _coeff)

    def friction_coeff(self, ):
        return papi.bodynode__getFrictionCoeff(self.wid, self.skid, self.id)

    def set_restitution_coeff(self, _coeff):
        papi.bodynode__setRestitutionCoeff(self.wid,
                                           self.skid,
                                           self.id,
                                           _coeff)

    def restitution_coeff(self, ):
        return papi.bodynode__getRestitutionCoeff(self.wid, self.skid, self.id)

########################################
# Transform Functions
    def transform(self, ):
        return papi.bodynode__getTransform(self.wid, self.skid, self.id)

    @property
    def T(self):
        return self.transform()

    def to_world(self, x=None):
        if x is None:
            x = [0.0, 0.0, 0.0]
        x_ = np.append(x, [1.0])
        return (self.T.dot(x_))[:3]

    def to_local(self, x):
        T = self.transform()
        return T[:3, :3].T.dot(np.asarray(x)-T[:3, 3])
        # Tinv = np.linalg.inv(self.T)
        # x_ = np.append(x, [1.0])
        # return (Tinv.dot(x_))[:3]

    def world_transform(self, ):
        """Get the transform of this Frame with respect to the World Frame.
        :rtype: np.ndarray
        """
        return papi.bodynode__getWorldTransform(self.wid, self.skid, self.id)

    def relative_transform(self, ):
        """
        Get the transform of this BodyNode with respect to its parent BodyNode, which is also its parent Frame.
        Implements dart::dynamics::Frame.
        :rtype: np.ndarray
        """
        return papi.bodynode__getRelativeTransform(self.wid,
                                                   self.skid,
                                                   self.id)


########################################
# Velocities and Accelerations Functions
    def world_angular_velocity(self):
        """
        Get angular velocity of this BodyNode expressed world frame
        :rtype: np.ndarray
        """
        return papi.bodynode__getAngularVelocity(self.wid, self.skid, self.id)

    def world_linear_velocity(self, offset=None):
        """
        Get angular velocity of this BodyNode expressed world frame
        :rtype: np.ndarray
        """
        if offset is None:
            offset = np.zeros(3)
        return papi.bodynode__getLinearVelocity(self.wid, self.skid, self.id, offset)

    def world_spatial_velocity(self, offset=None):
        """
        Get angular velocity of this BodyNode expressed world frame
        :rtype: np.ndarray
        """
        if offset is None:
            offset = np.zeros(3)
        return papi.bodynode__getSpatialVelocity(self.wid, self.skid, self.id, offset)

    def world_angular_acceleration(self):
        """
        Get angular velocity of this BodyNode expressed world frame
        :rtype: np.ndarray
        """
        return papi.bodynode__getAngularAcceleration(self.wid, self.skid, self.id)

    def world_linear_acceleration(self, offset=None):
        """
        Get angular velocity of this BodyNode expressed world frame
        :rtype: np.ndarray
        """
        if offset is None:
            offset = np.zeros(3)
        return papi.bodynode__getLinearAcceleration(self.wid, self.skid, self.id, offset)

    def world_spatial_acceleration(self, offset=None):
        """
        Get angular velocity of this BodyNode expressed world frame
        :rtype: np.ndarray
        """
        if offset is None:
            offset = np.zeros(3)
        return papi.bodynode__getSpatialAcceleration(self.wid, self.skid, self.id, offset)


########################################
# Torque Functions
    def add_ext_force(self,
                      _force,
                      _offset=None,
                      _isForceLocal=False,
                      _isOffsetLocal=True):
        if _offset is None:
            _offset = np.zeros(3)
        papi.bodynode__addExtForce(self.wid,
                                   self.skid,
                                   self.id,
                                   _force,
                                   _offset,
                                   _isForceLocal,
                                   _isOffsetLocal)

    def set_ext_force(self,
                      _force,
                      _offset=None,
                      _isForceLocal=False,
                      _isOffsetLocal=True):
        if _offset is None:
            _offset = np.zeros(3)
        papi.bodynode__setExtForce(self.wid,
                                   self.skid,
                                   self.id,
                                   _force,
                                   _offset,
                                   _isForceLocal,
                                   _isOffsetLocal)

    def add_ext_torque(self, _torque, _isLocal=False):
        papi.bodynode__addExtTorque(self.wid, self.skid, self.id,
                                    _torque, _isLocal)

    def set_ext_torque(self, _torque, _isLocal=False):
        papi.bodynode__setExtTorque(self.wid, self.skid, self.id,
                                    _torque, _isLocal)

########################################
# Jacobian Functions
    def jacobian(self, offset=None, full=True):
        """
        Return the generalized Jacobian targeting the origin of this BodyNode.

        The Jacobian is expressed in the Frame of this BodyNode.
        :type offset: np.ndarray
        :type full: bool
        :rtype: np.ndarray
        """
        offset = np.zeros(3) if offset is None else offset
        J = np.zeros((6, len(self.dependent_dofs)))
        papi.bodynode__getJacobian(self.wid,
                                   self.skid,
                                   self.id,
                                   offset,
                                   J)
        return self.expand_jacobian(J) if full else J

    def linear_jacobian(self, offset=None, full=True):
        """
        Return the linear Jacobian targeting an offset within the Frame of this BodyNode.
        Expressed in the world Frame
        :type offset: np.ndarray
        :type full: bool
        :rtype: np.ndarray
        """
        offset = np.zeros(3) if offset is None else offset
        J = np.zeros((3, len(self.dependent_dofs)))
        papi.bodynode__getLinearJacobian(self.wid,
                                         self.skid,
                                         self.id,
                                         offset,
                                         J)
        return self.expand_jacobian(J) if full else J

    def angular_jacobian(self, full=True):
        """
        Return the angular Jacobian targeting the origin of this BodyNode
        within the Frame of this BodyNode.
        Expressed in the worldFrame
        :type full: bool
        :rtype: np.ndarray
        """
        J = np.zeros((3, len(self.dependent_dofs)))
        papi.bodynode__getAngularJacobian(self.wid, self.skid, self.id, J)
        return self.expand_jacobian(J) if full else J

    def world_jacobian(self, offset=None, full=True):
        """
        Return the generalized Jacobian targeting an offset in this JacobianNode.

        The offset is expected in coordinates of this BodyNode Frame. The Jacobian is expressed in the World Frame.
        angular first, linear followed
        :type offset: np.ndarray
        :type full: bool
        :rtype: np.ndarray
        """
        offset = np.zeros(3) if offset is None else offset
        J = np.zeros((6, len(self.dependent_dofs)))
        papi.bodynode__getWorldJacobian(self.wid,
                                        self.skid,
                                        self.id,
                                        offset,
                                        J)
        return self.expand_jacobian(J) if full else J

    def linear_jacobian_deriv(self, offset=None, full=True):
        offset = np.zeros(3) if offset is None else offset
        J = np.zeros((3, len(self.dependent_dofs)))
        papi.bodynode__getLinearJacobianDeriv(self.wid,
                                              self.skid,
                                              self.id,
                                              offset,
                                              J)
        return self.expand_jacobian(J) if full else J

    def angular_jacobian_deriv(self, full=True):
        J = np.zeros((3, len(self.dependent_dofs)))
        papi.bodynode__getAngularJacobianDeriv(self.wid, self.skid, self.id, J)
        return self.expand_jacobian(J) if full else J

    def world_jacobian_classic_deriv(self, offset=None, full=True):
        offset = np.zeros(3) if offset is None else offset
        J = np.zeros((6, len(self.dependent_dofs)))
        papi.bodynode__getJacobianClassicDeriv(self.wid,
                                               self.skid,
                                               self.id,
                                               offset,
                                               J)
        return self.expand_jacobian(J) if full else J

    def expand_jacobian(self, jacobian):
        J = jacobian
        F = np.zeros((J.shape[0], self.skel.ndofs))
        I = np.array([dof.index for dof in self.dependent_dofs])
        F[:, I] = J
        return F

    @property
    def J(self, ):
        return self.linear_jacobian()

    @property
    def dJ(self, ):
        return self.linear_jacobian_deriv()

    def __repr__(self):
        return '[BodyNode(%d): %s]' % (self.id, self.name)
