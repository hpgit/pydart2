#include <iostream>
#include <string>
#include <vector>
#include <map>
using std::cout;
using std::cerr;
using std::endl;

// Boost headers
#include <boost/algorithm/string.hpp>

#include "pydart2_manager.h"
#include "pydart2_api.h"
#include "pydart2_skeleton_api.h"
#include "pydart2_draw.h"

using namespace pydart;


////////////////////////////////////////////////////////////////////////////////
// Skeleton
void SKEL(render)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawSkeleton(ri, skel.get());
}

void SKEL(renderWithColor)(int wid, int skid, double inv4[4]) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    Eigen::Vector4d color(inv4);
    // MSG << "color = " << color.transpose() << "\n";
    drawSkeleton(ri, skel.get(), color, false);
}

const char* SKEL(getName)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getName().c_str();
}

double SKEL(getMass)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getMass();
}

////////////////////////////////////////
// Skeleton::Property Functions
bool SKEL(isMobile)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->isMobile();
}

void SKEL(setMobile)(int wid, int skid, bool mobile) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setMobile(mobile);
}

bool SKEL(getSelfCollisionCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getSelfCollisionCheck();
}

void SKEL(setSelfCollisionCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setSelfCollisionCheck(enable);
}

bool SKEL(getAdjacentBodyCheck)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getAdjacentBodyCheck();
}

void SKEL(setAdjacentBodyCheck)(int wid, int skid, int enable) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setAdjacentBodyCheck(enable);
}

void SKEL(setRootJointToTransAndEuler)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    // change the joint type to euler
    dart::dynamics::BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<dart::dynamics::EulerJoint>();
    oldRoot->getParentJoint()->setName("root_r");
    // create a new root
    std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> ret =
        skel->createJointAndBodyNodePair
        <dart::dynamics::TranslationalJoint, dart::dynamics::BodyNode>();
    dart::dynamics::Joint* newJoint = ret.first;
    newJoint->setName("root_t");
    dart::dynamics::BodyNode* newBody = ret.second;
    newBody->setMass(0.0);
    // rearrange the root joints
    oldRoot->moveTo(newBody);
}


void SKEL(setRootJointToWeld)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    // change the joint type to euler
    dart::dynamics::BodyNode* oldRoot = skel->getRootBodyNode();
    oldRoot->changeParentJointType<dart::dynamics::WeldJoint>();

    // oldRoot->getParentJoint()->setName("root_r");
    // // create a new root
    // std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> ret =
    //     skel->createJointAndBodyNodePair
    //     <dart::dynamics::TranslationalJoint, dart::dynamics::BodyNode>();
    // dart::dynamics::Joint* newJoint = ret.first;
    // newJoint->setName("root_t");
    // dart::dynamics::BodyNode* newBody = ret.second;
    // newBody->setMass(0.0);
    // // rearrange the root joints
    // oldRoot->moveTo(newBody);
}


////////////////////////////////////////
// Skeleton::Structure Information Functions
int SKEL(getNumBodyNodes)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumBodyNodes();
}

int SKEL(getNumJoints)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumJoints();
}

int SKEL(getNumDofs)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumDofs();
}

int SKEL(getNumMarkers)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    return skel->getNumMarkers();
}

////////////////////////////////////////
// Skeleton::Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getPositions(), outv);
}

void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setPositions(read(inv, ndofs));
}

void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getVelocities(), outv);
}

void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setVelocities(read(inv, ndofs));
}

void SKEL(getAccelerations)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getAccelerations(), outv);
}

void SKEL(setForces)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setForces(read(inv, ndofs));
}

void SKEL(setAccelerations)(int wid, int skid, double* inv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setAccelerations(read(inv, ndofs));
}

////////////////////////////////////////
// Skeleton::Difference Functions
void SKEL(getVelocityDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd dq1 = read(inv1, indofs1);
    Eigen::VectorXd dq2 = read(inv2, indofs2);
    Eigen::VectorXd dq_diff = skel->getVelocityDifferences(dq1, dq2);
    write(dq_diff, outv);
}

void SKEL(getPositionDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd q1 = read(inv1, indofs1);
    Eigen::VectorXd q2 = read(inv2, indofs2);
    Eigen::VectorXd q_diff = skel->getPositionDifferences(q1, q2);
    write(q_diff, outv);
}

////////////////////////////////////////
// Skeleton::Limit Functions
void SKEL(getPositionLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionLowerLimits(), outv);
}

void SKEL(getPositionUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getPositionUpperLimits(), outv);
}

void SKEL(getForceLowerLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceLowerLimits(), outv);
}

void SKEL(getForceUpperLimits)(int wid, int skid, double* outv, int ndofs) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getForceUpperLimits(), outv);
}

////////////////////////////////////////
// Skeleton::Momentum Functions
void SKEL(getCOM)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOM(), outv3);
}

void SKEL(getCOMLinearVelocity)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearVelocity(), outv3);
}

void SKEL(getCOMLinearAcceleration)(int wid, int skid, double outv3[3]) {
   dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
   write(skel->getCOMLinearAcceleration(), outv3);
}

void SKEL(getCOMSpatialVelocity)(int wid, int skid, double outv6[6]) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getCOMSpatialVelocity(), outv6);
}

////////////////////////////////////////
// Skeleton::Lagrangian Functions
void SKEL(getMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write_matrix(skel->getMassMatrix(), outm);
}

void SKEL(getCoriolisAndGravityForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getCoriolisAndGravityForces(), outv);
}

void SKEL(getConstraintForces)(int wid, int skid, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write(skel->getConstraintForces(), outv);
}

void SKEL(getInvMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    write_matrix(skel->getInvMassMatrix(), outm);
}

////////////////////////////////////////
// Skeleton::PD Functions
void SKEL(getSPDForces)(int wid, int skid, double* inv1, int indofs1, double kp, double kd, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd tau = skel->getSPDForces(read(inv1, indofs1), kp, kd, GET_WORLD(wid)->getConstraintSolver());

    write(tau, outv);
}

void SKEL(setSPDTarget)(int wid, int skid, double kp, double kd, double* inv1, int indofs1) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->setSPDTarget(read(inv1, indofs1), kp, kd);
}

void SKEL(getSimplePDForces)(int wid, int skid, double h, double kp, double kd, double* inv1, int indofs1, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd dq = skel->getVelocities();

    //TODO:
    // compute skel submass tree

    Eigen::VectorXd tau = kp * (skel->getPositionDifferences(read(inv1, indofs1), skel->getPositions()) - h * dq) -  kd * dq;
    tau.head(6).setZero();

    write(tau, outv);
}

void SKEL(getStablePDForces)(int wid, int skid, double h, double kp, double kd, double* inv1, int indofs1, double* outv, int ndofs) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    double kd_h = kd * h;
    Eigen::VectorXd dq = skel->getVelocities();

    Eigen::VectorXd q_des = read(inv1, indofs1);
    Eigen::VectorXd q = skel->getPositions();
    q_des.head(6).setZero();
    q.head(6).setZero();

    Eigen::VectorXd p_d = kp * (skel->getPositionDifferences(q_des, q) - h * dq) -  kd * dq;
    p_d.head(6).setZero();
    Eigen::MatrixXd skel_identity = Eigen::MatrixXd::Identity(ndofs, ndofs);
    skel_identity.block<6,6>(0, 0).setZero();
    Eigen::VectorXd tau = p_d - kd_h * (skel->getMassMatrix() + kd_h * skel_identity)
            .llt().solve(p_d - skel->getCoriolisAndGravityForces() + skel->getConstraintForces());

    tau.head(6).setZero();

    write(tau, outv);
}

void SKEL(getStablePDForcesExtended)(int wid, int skid, double h, double* inv1, int indofs1, double* inv2, int indofs2, double* inv3, int indofs3, double* outv, int ndofs){
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd dq = skel->getVelocities();
    Eigen::VectorXd Kp = read(inv1, indofs1);
    Eigen::VectorXd Kd = read(inv2, indofs2);

    Eigen::VectorXd p_d = Kp.cwiseProduct(skel->getPositionDifferences(read(inv3, indofs3), skel->getPositions()) - h * dq) -  Kd.cwiseProduct(dq);
    Eigen::VectorXd tau = p_d - h * Kd.cwiseProduct(
            (skel->getMassMatrix() + h * Eigen::MatrixXd(Kd.asDiagonal())).llt().solve(p_d - skel->getCoriolisAndGravityForces() + skel->getConstraintForces()));
//    Eigen::VectorXd tau = p_d - h * Kd.cwiseProduct(
//            (skel->getMassMatrix() + h * Eigen::MatrixXd(Kd.asDiagonal())).lu().solve(p_d - skel->getCoriolisAndGravityForces() + skel->getConstraintForces()));

    tau.head(6).setZero();

    write(tau, outv);
}

void SKEL(getSimpleStablePDForcesExtended)(int wid, int skid, double h, double* inv0, int indofs0, double* inv1, int indofs1, double* inv2, int indofs2, double* inv3, int indofs3, double* outv, int ndofs){
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::VectorXd dq = skel->getVelocities();
    Eigen::VectorXd mass_matrix_eig = read(inv0, indofs0);
    Eigen::VectorXd Kp = read(inv1, indofs1);
    Eigen::VectorXd Kd = read(inv2, indofs2);

    Eigen::VectorXd p_d = Kp.cwiseProduct(skel->getPositionDifferences(read(inv3, indofs3), skel->getPositions()) - h * dq) -  Kd.cwiseProduct(dq);
    std::cout << "mass_matrix_eig : " << mass_matrix_eig << std::endl;
    std::cout << "mass_eig edit : " << (mass_matrix_eig + h * Kd).cwiseInverse() << std::endl;

    Eigen::VectorXd tau = p_d - h * Kd.cwiseProduct(
                    (p_d - skel->getCoriolisAndGravityForces() + skel->getConstraintForces()).cwiseProduct((mass_matrix_eig + h * Kd).cwiseInverse()));
    std::cout << "tau : " << tau << std::endl;

    Eigen::VectorXd tau_ref = p_d - h * Kd.cwiseProduct(
            (skel->getMassMatrix() + h * Eigen::MatrixXd(Kd.asDiagonal())).llt().solve(p_d - skel->getCoriolisAndGravityForces() + skel->getConstraintForces()));
    std::cout << "tau_ref : " << tau_ref << std::endl;


    tau.head(6).setZero();

    write(tau, outv);
}


void SKEL(addBox)(int wid, int skid, const char * const name, double inv3[3], double inv3_2[3]) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::Vector3d box_size = read(inv3, 3);
    Eigen::Vector3d box_color = read(inv3_2, 3);

    dart::dynamics::BodyNodePtr ground = skel->getBodyNode(0);
    Eigen::Isometry3d j_t_p = ground->getWorldTransform();

    dart::dynamics::FreeJoint::Properties props;

	props.mName = name;
	props.mT_ParentBodyToJoint = j_t_p.inverse();
	props.mT_ChildBodyToJoint = Eigen::Isometry3d::Identity();
	props.mIsPositionLimitEnforced = false;

    dart::dynamics::ShapePtr box_shape = std::make_shared<dart::dynamics::BoxShape>(box_size);
    dart::dynamics::BodyNodePtr bn = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        ground, props, dart::dynamics::BodyNode::AspectProperties(name) 
    ).second;

    bn->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box_shape);
    bn->getShapeNodesWith<dart::dynamics::VisualAspect>().back()->getVisualAspect()->setColor(box_color);
}

void SKEL(setBox)(int wid, int skid, const char * const name, double inv3[3]) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    Eigen::Vector3d box_size = read(inv3, 3);

    dart::dynamics::BodyNodePtr bn = skel->getBodyNode(std::string(name));
    dynamic_cast<dart::dynamics::BoxShape*>(bn->getShapeNodesWith<dart::dynamics::VisualAspect>().back()->getShape().get())->setSize(box_size);
    dynamic_cast<dart::dynamics::BoxShape*>(bn->getShapeNodesWith<dart::dynamics::DynamicsAspect>().back()->getShape().get())->setSize(box_size);
    dynamic_cast<dart::dynamics::BoxShape*>(bn->getShapeNodesWith<dart::dynamics::CollisionAspect>().back()->getShape().get())->setSize(box_size);
}

void SKEL(clearImpulse)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->clearConstraintImpulses();
    skel->clearInternalForces();
    skel->clearExternalForces();
}

void SKEL(computeForwardKinematics)(int wid, int skid) {
    dart::dynamics::SkeletonPtr skel = GET_SKELETON(wid, skid);
    skel->computeForwardKinematics(true, false, false);
}