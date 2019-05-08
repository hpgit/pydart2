//
// Created by Hwangpil Park on 2019-04-09.
//

#include <iostream>
#include "NonHolonomicContactConstraint_v2.h"

#include "dart/external/odelcpsolver/lcp.h"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

//#define HP_DEBUG

namespace dart {
    namespace constraint {
        NonHolonomicContactConstraintV2::NonHolonomicContactConstraintV2(dart::dynamics::BodyNode *_body,
                                                                    const Eigen::Vector3d &offsetOnBodyCoord)
                 :JointConstraint(_body),
                mOffset1(offsetOnBodyCoord),
                mOffset2(Eigen::Vector3d::Zero()),
                mBodyPos(Eigen::Vector3d::Zero()),
                mViolation(0.),
                mDesiredProjectedVector(Eigen::Vector3d(1., 0, 0)),
                mAppliedImpulseIndex(0),
                bActive(false),
                dViolationAngleIgnoreThreshold(0.)
        {
            mDim = 1;

            mOldX[0] = 0.0;

            mJacobian1.setZero();
            mJacobian1(0, 5) = 1.;
        }

        void NonHolonomicContactConstraintV2::setPrevBodyNodePos(const Eigen::Vector3d &_bodyPos) {
            mPrevBodyPos = _bodyPos;
        }

        void NonHolonomicContactConstraintV2::update() {
            // mBodyNode1 should not be null pointer ever
            assert(mBodyNode1);
#ifdef HP_DEBUG
            std::cout << "update()" << std::endl;
#endif

//            Eigen::Vector3d body_projected_vector = mBodyNode1->getTransform().linear() * Eigen::Vector3d(1., 0., 0.);
//            body_projected_vector(1) = 0.;
//            body_projected_vector.normalize();
//
//            if (body_projected_vector.dot(mDesiredProjectedVector) < dViolationAngleIgnoreThreshold)
//            {
//                bActive = false;
//                return;
//            }

            mDesiredProjectedVector = mBodyNode1->getTransform() * Eigen::Vector3d::(0., 0., 0.) - mPrevBodyPos;
            mDesiredProjectedVector[1] = 0.;
            if (mDesiredProjectedVector.norm() < 0.000001)
            {
                mDesiredProjectedVector = mBodyNode1->getTransform() * mOffset1 - mPrevBodyPos;
                mDesiredProjectedVector[1] = 0.;
            }
            mDesiredProjectedVector.normalize();

            mOffset2 = mBodyNode1->getTransform() * mOffset1;
            mOffset2[1] = 0.;


            // Jacobian update
            Eigen::Vector3d localProjectedPerpVector = mBodyNode1->getTransform().linear().inverse() * mDesiredProjectedVector.cross(Eigen::Vector3d(0., 1., 0.));
            mJacobian1.setZero();
            mJacobian1.block<1, 3>(0, 0) = mOffset1.cross(localProjectedPerpVector);
            mJacobian1.block<1, 3>(0, 3) = localProjectedPerpVector;

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
                mViolation = localProjectedPerpVector.dot(mOffset1 - mBodyNode1->getTransform().inverse() * mOffset2);
            }

            //  std::cout << "mViolation = " << mViolation << std::endl;
        }

        void NonHolonomicContactConstraintV2::getInformation(ConstraintInfo *_lcp) {
#ifdef HP_DEBUG
            std::cout << "getInformation()" << std::endl;
#endif
            assert(_lcp->w[0] == 0.0);

            assert(_lcp->findex[0] == -1);

            _lcp->lo[0] = -dInfinity;

            _lcp->hi[0] = dInfinity;

            Eigen::VectorXd negativeVel = -mJacobian1 * mBodyNode1->getSpatialVelocity();

            mViolation *= mErrorReductionParameter * _lcp->invTimeStep;

            _lcp->b[0] = negativeVel[0] - mViolation;

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

            Eigen::VectorXd imp(1);
            imp(0) = _lambda[0];

            // std::cout << "lambda: " << _lambda[0] << " " << _lambda[1] << " " << _lambda[2] << std::endl;

            mBodyNode1->addConstraintImpulse(mJacobian1.transpose() * imp);

            if (mBodyNode2)
                mBodyNode2->addConstraintImpulse(-mJacobian2.transpose() * imp);
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

