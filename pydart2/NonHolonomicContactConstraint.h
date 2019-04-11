//
// Created by Hwangpil Park on 2019-04-09.
//

#ifndef DART_NONHOLONOMICCONTACTCONSTRAINT_HPP
#define DART_NONHOLONOMICCONTACTCONSTRAINT_HPP

#include <Eigen/Dense>

#include "dart/constraint/JointConstraint.hpp"

namespace dart{
    namespace constraint{
        class NonHolonomicContactConstraint : public JointConstraint {
        public:
            /// Constructor that takes one body and the joint position in the world frame
            /// \param[in] _jointPos Joint position expressed in world frame
            NonHolonomicContactConstraint(dynamics::BodyNode* _body, const Eigen::Vector3d &offsetOnBodyCoord);

            ~NonHolonomicContactConstraint() override {}

            void setJointPos(const Eigen::Vector3d& _jointPos);

            void setProjectedVector(const Eigen::Vector3d& _projectedVector);

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

            void setViolationAngleIgnoreThreshold(double _threshold){dViolationAngleIgnoreThreshold = _threshold;}

            void setLengthForViolationIgnore(double _length){dLengthForViolationIgnore = _length;}

        private:
            /// be forced activation
            bool bActive;

            /// threshold for ignoring violation expressed in radian
            double dViolationAngleIgnoreThreshold;

            /// bodynode length for ignoring violation
            /// bodynode must have two nonholonomic points
            /// this variable express distance between two points
            double dLengthForViolationIgnore;

            /// Offset from the origin of body frame 1 to the ball joint position where
            /// the offset is expressed in body frame 1
            Eigen::Vector3d mOffset1;

            /// Offset from the origin of body frame 2 to the ball joint position where
            /// the offset is expressed in body frame 2
            Eigen::Vector3d mOffset2;

            /// projected vector expressed in WorldFrame
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


        DART_COMMON_DECLARE_SHARED_WEAK(NonHolonomicContactConstraint)

    }
}




#endif //DART_NONHOLONOMICCONTACTCONSTRAINT_HPP
