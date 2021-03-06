//
// Created by Hwangpil Park on 2019-04-09.
//

#include <iostream>
#include "NonHolonomicContactConstraint_v2.h"

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

//#define HP_DEBUG

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif // _USE_MATH_DEFINES

namespace dart {
    namespace constraint {
        NonHolonomicContactConstraintV2::NonHolonomicContactConstraintV2(dart::dynamics::BodyNode *_body,
                                                                    const Eigen::Vector3d& offsetOnBodyCoord1,
                                                                    const Eigen::Vector3d& offsetOnBodyCoord3)
                 :JointConstraint(_body),
                  bActive(false),
                  dViolationAngleIgnoreThreshold(0.),
                mOffset1(offsetOnBodyCoord1),
                mOffset2(Eigen::Vector3d::Zero()),
                mOffset3(offsetOnBodyCoord3),
                mOffset4(Eigen::Vector3d::Zero()),
                mPrevBodyPos(Eigen::Vector3d::Zero()),
                mPrevBodyVec(Eigen::Vector3d(1., 0., 0.)),
                mDesiredProjectedVector(Eigen::Vector3d(1., 0., 0.)),
                mAppliedImpulseIndex(0)
        {
            mDim = 2;

            mOldX[0] = 0.0;
            mOldX[1] = 0.0;

            mViolation[0] = 0.0;
            mViolation[1] = 0.0;

            mJacobian1.setZero();
            mJacobian1(0, 5) = 1.;
            mJacobian1(1, 5) = 1.;
            mErrorReductionParameter = 0.005;
        }

        void NonHolonomicContactConstraintV2::setPrevBodyNodePos(const Eigen::Vector3d &_bodyPos) {
            mPrevBodyPos = _bodyPos;
        }

        void NonHolonomicContactConstraintV2::setPrevBodyNodeVec(const Eigen::Vector3d &_bodyVec) {
            mPrevBodyVec = _bodyVec;
            mPrevBodyVec[1] = 0.;
            mPrevBodyVec.normalize();
        }

        void NonHolonomicContactConstraintV2::update() {
            // mBodyNode1 should not be null pointer ever
            assert(mBodyNode1);
#ifdef HP_DEBUG
            std::cout << "update()" << std::endl;
#endif

            // calculate direction vector
            Eigen::Vector3d ori_direction = mBodyNode1->getTransform().translation() - mPrevBodyPos;
            ori_direction[1] = 0.;
//            std::cout << "ori_direction: " << ori_direction << std::endl;
            mDesiredProjectedVector = ori_direction;
//            std::cout << "1mDesiredProjectedVector: " << mDesiredProjectedVector << std::endl;

            if (mDesiredProjectedVector.norm() < 0.000001)
            {
//                mDesiredProjectedVector = mBodyNode1->getTransform() * mOffset1 - mPrevBodyPos;
                mDesiredProjectedVector = mPrevBodyVec;
//                std::cout << "2mDesiredProjectedVector: " << mDesiredProjectedVector << std::endl;
            }
            mDesiredProjectedVector.normalize();
//            std::cout << "3mDesiredProjectedVector: " << mDesiredProjectedVector << std::endl;

            double violated_angle = acos(mDesiredProjectedVector.dot(mPrevBodyVec));
//            std::cout << "violated_angle: " << violated_angle << std::endl;
            if (violated_angle > dViolationAngleIgnoreThreshold && violated_angle < M_PI - dViolationAngleIgnoreThreshold ) {
                // violated angle case
                int min_violated_idx = 0;
                double max_violated_cosval = -1.;
                Eigen::Vector3d dir[4];
                dir[0] = Eigen::AngleAxisd(dViolationAngleIgnoreThreshold, Eigen::Vector3d(0., 1., 0.)) * mPrevBodyVec;
                dir[1] = Eigen::AngleAxisd(-dViolationAngleIgnoreThreshold, Eigen::Vector3d(0., 1., 0.)) * mPrevBodyVec;
                dir[2] = -dir[0];
                dir[3] = -dir[1];
                for(int i=0; i<4; i++) {
                    double value = ori_direction.normalized().dot(dir[i]);
                    if(value > max_violated_cosval)
                    {
                        max_violated_cosval = value;
                        min_violated_idx = i;
                    }
                }
//                std::cout << "min_violated_idx: " << min_violated_idx<< std::endl;
//                std::cout << "max_violated_cosval: " << max_violated_cosval << std::endl;
//                std::cout << "dir[0]" << dir[0] << std::endl;
//                std::cout << "dir[1]" << dir[1] << std::endl;
//                std::cout << "dir[2]" << dir[2] << std::endl;
//                std::cout << "dir[3]" << dir[3] << std::endl;
                mDesiredProjectedVector = dir[min_violated_idx];
            }
//            std::cout << "4mDesiredProjectedVector: " << mDesiredProjectedVector << std::endl;

            // set joint pos
            Eigen::Vector3d projectedOffset1 = mOffset1;
            projectedOffset1[1] = 0.;
//            std::cout << "ori_direction.norm: " << ori_direction.norm() << std::endl;
            if (mDesiredProjectedVector.dot(mPrevBodyVec) > 0.) {
                mOffset2 = mPrevBodyPos + (ori_direction.norm() + projectedOffset1.norm()) * mDesiredProjectedVector;
            }
            else {
                mOffset2 = mPrevBodyPos + (ori_direction.norm() - projectedOffset1.norm()) * mDesiredProjectedVector;
            }
            mOffset2[1] = 0.;
            std::cout << "mOffset2: " << mOffset2 << std::endl;

            Eigen::Vector3d projectedOffset3 = mOffset3;
            projectedOffset3[1] = 0.;
            if (mDesiredProjectedVector.dot(mPrevBodyVec) > 0.) {
                mOffset4 = mPrevBodyPos + (ori_direction.norm() - projectedOffset3.norm()) * mDesiredProjectedVector;
            }
            else {
                mOffset4 = mPrevBodyPos + (ori_direction.norm() + projectedOffset3.norm()) * mDesiredProjectedVector;
            }
            mOffset4[1] = 0.;
            std::cout << "mOffset4: " << mOffset4 << std::endl;

            // Jacobian update
            Eigen::Vector3d localProjectedPerpVector = mBodyNode1->getTransform().linear().inverse() * mDesiredProjectedVector.cross(Eigen::Vector3d(0., 1., 0.));
            mJacobian1.setZero();
            mJacobian1.block<1, 3>(0, 0) = mOffset1.cross(localProjectedPerpVector);
            mJacobian1.block<1, 3>(0, 3) = localProjectedPerpVector;
            mJacobian1.block<1, 3>(1, 0) = mOffset3.cross(localProjectedPerpVector);
            mJacobian1.block<1, 3>(1, 3) = localProjectedPerpVector;

            // Update Jacobian for body2
            //TODO:
            if (mBodyNode2)
            {
                /*
                Eigen::Isometry3d T12 = mBodyNode1->getTransform().inverse()
                                        * mBodyNode2->getTransform();
                Eigen::Vector3d p2 = T12.inverse() * mOffset1;

                Eigen::Matrix<double, 3, 6> J2;
                J2.leftCols<3>()  = math::makeSkewSymmetric(-p2);
                J2.rightCols<3>() = Eigen::Matrix3d::Identity();

                mJacobian2 = T12.linear() * J2;
                 */
            }

            // Update position constraint error
            //TODO:
            if (mBodyNode2)
            {
                //mViolation = mOffset1 - mBodyNode1->getTransform().inverse()
                //                        * mBodyNode2->getTransform() * mOffset2;
            }
            else
            {
                mViolation[0] = localProjectedPerpVector.dot(mOffset1 - mBodyNode1->getTransform().inverse() * mOffset2);
                mViolation[1] = localProjectedPerpVector.dot(mOffset3 - mBodyNode1->getTransform().inverse() * mOffset4);
            }

            //  std::cout << "mViolation = " << mViolation << std::endl;
        }

        void NonHolonomicContactConstraintV2::getInformation(ConstraintInfo *_lcp) {
#ifdef HP_DEBUG
            std::cout << "getInformation()" << std::endl;
#endif
            assert(_lcp->w[0] == 0.0);
            assert(_lcp->w[1] == 0.0);

            assert(_lcp->findex[0] == -1);
            assert(_lcp->findex[1] == -1);

            _lcp->lo[0] = -dInfinity;
            _lcp->lo[1] = -dInfinity;

            _lcp->hi[0] = dInfinity;
            _lcp->hi[1] = dInfinity;

            Eigen::VectorXd negativeVel = (-mJacobian1) * mBodyNode1->getSpatialVelocity();

            mViolation[0] *= mErrorReductionParameter * _lcp->invTimeStep;
            mViolation[1] *= mErrorReductionParameter * _lcp->invTimeStep;

            _lcp->b[0] = negativeVel[0] - mViolation[0];
            _lcp->b[1] = negativeVel[1] - mViolation[1];

        }

        void NonHolonomicContactConstraintV2::applyUnitImpulse(std::size_t _index) {
#ifdef HP_DEBUG
            std::cout << "applyUnitImpulse()" << std::endl;
#endif
            assert(_index < mDim && "Invalid Index.");
            assert(isActive());

            if (mBodyNode2)
            {
                assert(mBodyNode1->isReactive() || mBodyNode2->isReactive());

                // Self collision case
                if (mBodyNode1->getSkeleton() == mBodyNode2->getSkeleton())
                {
                    mBodyNode1->getSkeleton()->clearConstraintImpulses();

                    if (mBodyNode1->isReactive())
                    {
                        if (mBodyNode2->isReactive())
                        {
                            mBodyNode1->getSkeleton()->updateBiasImpulse(
                                    mBodyNode1, mJacobian1.row(_index),
                                    mBodyNode2, -mJacobian2.row(_index));
                        }
                        else
                        {
                            mBodyNode1->getSkeleton()->updateBiasImpulse(
                                    mBodyNode1, mJacobian1.row(_index));
                        }
                    }
                    else
                    {
                        if (mBodyNode2->isReactive())
                        {
                            mBodyNode2->getSkeleton()->updateBiasImpulse(
                                    mBodyNode2, -mJacobian2.row(_index));
                        }
                        else
                        {
                            assert(0);
                        }
                    }
                    mBodyNode1->getSkeleton()->updateVelocityChange();
                }
                    // Colliding two distinct skeletons
                else
                {
                    if (mBodyNode1->isReactive())
                    {
                        mBodyNode1->getSkeleton()->clearConstraintImpulses();
                        mBodyNode1->getSkeleton()->updateBiasImpulse(
                                mBodyNode1, mJacobian1.row(_index));
                        mBodyNode1->getSkeleton()->updateVelocityChange();
                    }

                    if (mBodyNode2->isReactive())
                    {
                        mBodyNode2->getSkeleton()->clearConstraintImpulses();
                        mBodyNode2->getSkeleton()->updateBiasImpulse(
                                mBodyNode2, -mJacobian2.row(_index));
                        mBodyNode2->getSkeleton()->updateVelocityChange();
                    }
                }
            }
            else
            {
                assert(mBodyNode1->isReactive());

                mBodyNode1->getSkeleton()->clearConstraintImpulses();
                mBodyNode1->getSkeleton()->updateBiasImpulse(
                        mBodyNode1, mJacobian1.row(_index));
                mBodyNode1->getSkeleton()->updateVelocityChange();
            }

            mAppliedImpulseIndex = _index;

        }

        void NonHolonomicContactConstraintV2::getVelocityChange(double *_vel, bool _withCfm) {
#ifdef HP_DEBUG
            std::cout << "getVelocityChange()" << std::endl;
#endif
            assert(_vel != nullptr && "Null pointer is not allowed.");

            for (std::size_t i = 0; i < mDim; ++i)
                _vel[i] = 0.0;

            if (mBodyNode1->getSkeleton()->isImpulseApplied()
                && mBodyNode1->isReactive())
            {
                Eigen::VectorXd v1 = mJacobian1 * mBodyNode1->getBodyVelocityChange();
                // std::cout << "velChange " << mBodyNode1->getBodyVelocityChange() << std::endl;
                // std::cout << "v1: " << v1 << std::endl;
                for (std::size_t i = 0; i < mDim; ++i)
                    _vel[i] += v1[i];
            }

            if (mBodyNode2
                && mBodyNode2->getSkeleton()->isImpulseApplied()
                && mBodyNode2->isReactive())
            {
                Eigen::VectorXd v2 = mJacobian2 * mBodyNode2->getBodyVelocityChange();
                // std::cout << "v2: " << v2 << std::endl;
                for (std::size_t i = 0; i < mDim; ++i)
                    _vel[i] -= v2[i];
            }

            // Add small values to diagnal to keep it away from singular, similar to cfm
            // varaible in ODE
            if (_withCfm)
            {
                _vel[mAppliedImpulseIndex] += _vel[mAppliedImpulseIndex]
                                              * mConstraintForceMixing;
            }

        }

        void NonHolonomicContactConstraintV2::excite() {
#ifdef HP_DEBUG
            std::cout << "excite()" << std::endl;
#endif
            if (mBodyNode1->isReactive())
                mBodyNode1->getSkeleton()->setImpulseApplied(true);

            if (mBodyNode2 == nullptr)
                return;

            if (mBodyNode2->isReactive())
                mBodyNode2->getSkeleton()->setImpulseApplied(true);
        }

        void NonHolonomicContactConstraintV2::unexcite() {
#ifdef HP_DEBUG
            std::cout << "unexcite()" << std::endl;
#endif
            if (mBodyNode1->isReactive())
                mBodyNode1->getSkeleton()->setImpulseApplied(false);

            if (mBodyNode2 == nullptr)
                return;

            if (mBodyNode2->isReactive())
                mBodyNode2->getSkeleton()->setImpulseApplied(false);
        }

        void NonHolonomicContactConstraintV2::applyImpulse(double *_lambda) {
#ifdef HP_DEBUG
            std::cout << "applyImpulse()" << std::endl;
#endif
            mOldX[0] = _lambda[0];
            mOldX[1] = _lambda[1];

            Eigen::VectorXd imp(2);
            imp(0) = _lambda[0];
            imp(1) = _lambda[1];

            // std::cout << "lambda: " << _lambda[0] << " " << _lambda[1] << " " << _lambda[2] << std::endl;

            mBodyNode1->addConstraintImpulse(mJacobian1.transpose() * imp);

//            if (mBodyNode2)
//                mBodyNode2->addConstraintImpulse(-mJacobian2.transpose() * imp);
        }

        dynamics::SkeletonPtr NonHolonomicContactConstraintV2::getRootSkeleton() const {
#ifdef HP_DEBUG
            std::cout << "getRootSkeleton()" << std::endl;
#endif
            if (mBodyNode1->isReactive())
                return mBodyNode1->getSkeleton()->mUnionRootSkeleton.lock();

            if (mBodyNode2)
            {
                if (mBodyNode2->isReactive())
                {
                    return mBodyNode2->getSkeleton()->mUnionRootSkeleton.lock();
                }
                else
                {
                    assert(0);
                    return nullptr;
                }
            }
            else
            {
                assert(0);
                return nullptr;
            }
        }

        void NonHolonomicContactConstraintV2::uniteSkeletons() {
#ifdef HP_DEBUG
            std::cout << "uniteSkeletons()" << std::endl;
#endif
            if (mBodyNode2 == nullptr)
                return;

            if (!mBodyNode1->isReactive() || !mBodyNode2->isReactive())
                return;

            if (mBodyNode1->getSkeleton() == mBodyNode2->getSkeleton())
                return;

            dynamics::SkeletonPtr unionId1
                    = ConstraintBase::compressPath(mBodyNode1->getSkeleton());
            dynamics::SkeletonPtr unionId2
                    = ConstraintBase::compressPath(mBodyNode2->getSkeleton());

            if (unionId1 == unionId2)
                return;

            if (unionId1->mUnionSize < unionId2->mUnionSize)
            {
                // Merge root1 --> root2
                unionId1->mUnionRootSkeleton = unionId2;
                unionId2->mUnionSize += unionId1->mUnionSize;
            }
            else
            {
                // Merge root2 --> root1
                unionId2->mUnionRootSkeleton = unionId1;
                unionId1->mUnionSize += unionId2->mUnionSize;
            }
        }

        bool NonHolonomicContactConstraintV2::isActive() const {
#ifdef HP_DEBUG
            std::cout << "isActive()" << std::endl;
#endif
            if (!bActive)
                return false;

            if (mBodyNode1->isReactive())
                return true;

            if (mBodyNode2)
            {
                if (mBodyNode2->isReactive())
                    return true;
                else
                    return false;
            }
            else
            {
                return false;
            }
        }

    }
}

