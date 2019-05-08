//
// Created by Hwangpil Park on 2019-04-09.
//

#ifndef DART_NONHOLONOMICCONTACTCONSTRAINTV2_HPP
#define DART_NONHOLONOMICCONTACTCONSTRAINTV2_HPP

#include <Eigen/Dense>

#include "dart/constraint/JointConstraint.hpp"

namespace dart{
    namespace constraint{
        class NonHolonomicContactConstraintV2 : public JointConstraint {
        public:
            /// Constructor that takes one body and the joint position in the world frame
            /// \param[in] _jointPos Joint position expressed in world frame
            NonHolonomicContactConstraintV2(dynamics::BodyNode* _body, const Eigen::Vector3d &offsetOnBodyCoord);

            ~NonHolonomicContactConstraintV2() override {}

            void update() override;

            void getInformation(ConstraintInfo *info) override;

            void applyUnitImpulse(std::size_t index) override;

            void getVelocityChange(double *vel, bool withCfm) override;

            void excite() override;

            void unexcite() override;

            void applyImpulse(double *lambda) override;

            bool isActive() const override;

            dynamics::SkeletonPtr getRootSkeleton() const override;

            void uniteSkeletons() override;

            void activate(bool _active){bActive = _active;}

            void setPrevBodyNodePos(const Eigen::Vector3d& _bodyPos);

            void setViolationAngleIgnoreThreshold(double _threshold){dViolationAngleIgnoreThreshold = _threshold;}

        private:
            /// be forced activation
            bool bActive;

            /// threshold for ignoring violation expressed in radian
            double dViolationAngleIgnoreThreshold;

            /// Offset from the origin of body frame 1 to the ball joint position where
            /// the offset is expressed in body frame 1
            Eigen::Vector3d mOffset1;

            /// Offset from the origin of body frame 2 to the ball joint position where
            /// the offset is expressed in body frame 2
            Eigen::Vector3d mOffset2;

            /// prev body1 position in World Frame
            Eigen::Vector3d mPrevBodyPos;

            /// projected vector expressed in World Frame
            Eigen::Vector3d mDesiredProjectedVector;

            /// Position constraint violation expressed in body frame 1
            double mViolation;

            /// Linear map between constraint space and Cartesian space for body1
            Eigen::Matrix<double, 1, 6> mJacobian1;

            /// Linear map between constraint space and Cartesian space for body2
            Eigen::Matrix<double, 1, 6> mJacobian2;

            ///
            double mOldX[1];

            /// Index of applied impulse
            std::size_t mAppliedImpulseIndex;

        public:
            // To get byte-aligned Eigen vectors
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };


        DART_COMMON_DECLARE_SHARED_WEAK(NonHolonomicContactConstraintV2)

    }
}




#endif //DART_NONHOLONOMICCONTACTCONSTRAINTV2_HPP
