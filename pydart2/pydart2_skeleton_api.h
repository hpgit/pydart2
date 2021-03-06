#ifndef PYDART2_SKELETON_API_H
#define PYDART2_SKELETON_API_H


////////////////////////////////////////////////////////////////////////////////
// Skeleton
#define SKEL(funcname) skeleton__##funcname
#define GET_WORLD(wid) Manager::world(wid)
#define GET_SKELETON(wid, skid) Manager::skeleton(wid, skid);

void SKEL(render)(int wid, int skid);
void SKEL(renderWithColor)(int wid, int skid, double inv4[4]);
const char* SKEL(getName)(int wid, int skid);
double SKEL(getMass)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Property Functions
bool SKEL(isMobile)(int wid, int skid);
void SKEL(setMobile)(int wid, int skid, bool mobile);
bool SKEL(getSelfCollisionCheck)(int wid, int skid);
void SKEL(setSelfCollisionCheck)(int wid, int skid, int enable);
bool SKEL(getAdjacentBodyCheck)(int wid, int skid);
void SKEL(setAdjacentBodyCheck)(int wid, int skid, int enable);
void SKEL(setRootJointToTransAndEuler)(int wid, int skid);
void SKEL(setRootJointToWeld)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Structure Information Functions
int SKEL(getNumBodyNodes)(int wid, int skid);
int SKEL(getNumJoints)(int wid, int skid);
int SKEL(getNumDofs)(int wid, int skid);
int SKEL(getNumMarkers)(int wid, int skid);

////////////////////////////////////////
// Skeleton::Pose Functions
void SKEL(getPositions)(int wid, int skid, double* outv, int ndofs);
void SKEL(setPositions)(int wid, int skid, double* inv, int ndofs);
void SKEL(getVelocities)(int wid, int skid, double* outv, int ndofs);
void SKEL(setVelocities)(int wid, int skid, double* inv, int ndofs);
void SKEL(getAccelerations)(int wid, int skid, double* outv, int ndofs);
void SKEL(setForces)(int wid, int skid, double* inv, int ndofs);
void SKEL(setAccelerations)(int wid, int skid, double* inv, int ndofs);

////////////////////////////////////////
// Skeleton::Difference Functions
void SKEL(getPositionDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs);
void SKEL(getVelocityDifferences)(int wid, int skid,
                                  double* inv1, int indofs1,
                                  double* inv2, int indofs2,
                                  double* outv, int ndofs);

////////////////////////////////////////
// Skeleton::Limit Functions
void SKEL(getPositionLowerLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getPositionUpperLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getForceLowerLimits)(int wid, int skid, double* outv, int ndofs);
void SKEL(getForceUpperLimits)(int wid, int skid, double* outv, int ndofs);

////////////////////////////////////////
// Skeleton::Momentum Functions
void SKEL(getCOM)(int wid, int skid, double outv3[3]);
void SKEL(getCOMLinearVelocity)(int wid, int skid, double outv3[3]);
void SKEL(getCOMLinearAcceleration)(int wid, int skid, double outv3[3]);
void SKEL(getCOMSpatialVelocity)(int wid, int skid, double outv6[6]);

////////////////////////////////////////
// Skeleton::Lagrangian Functions
void SKEL(getMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols);
void SKEL(getCoriolisAndGravityForces)(int wid, int skid, double* outv, int ndofs);
void SKEL(getConstraintForces)(int wid, int skid, double* outv, int ndofs);
void SKEL(getInvMassMatrix)(int wid, int skid, double* outm, int nrows, int ncols);

////////////////////////////////////////
// Skeleton::PD Functions
void SKEL(getSPDForces)(int wid, int skid, double* inv1, int indofs1, double kp, double kd, double* outv, int ndofs);
void SKEL(setSPDTarget)(int wid, int skid, double kp, double kd, double* inv1, int indofs1);
void SKEL(getSimplePDForces)(int wid, int skid, double h, double kp, double kd, double* inv1, int indofs1, double* outv, int ndofs);
void SKEL(getStablePDForces)(int wid, int skid, double h, double kp, double kd, double* inv1, int indofs1, double* outv, int ndofs);
void SKEL(getStablePDForcesExtended)(int wid, int skid, double h,
                            double* inv1, int indofs1,
                            double* inv2, int indofs2,
                            double* inv3, int indofs3,
                            double* outv, int ndofs);
void SKEL(getSimpleStablePDForcesExtended)(int wid, int skid, double h,
                                     double* inv0, int indofs0,
                                     double* inv1, int indofs1,
                                     double* inv2, int indofs2,
                                     double* inv3, int indofs3,
                                     double* outv, int ndofs);


////////////////////////////////////////
// Skeleton::AddBody Functions
void SKEL(addBox)(int wid, int skid, const char * const name, double inv3[3], double inv3_2[3]);
void SKEL(setBox)(int wid, int skid, const char * const name, double inv3[3]);

void SKEL(clearImpulse)(int wid, int skid);
void SKEL(computeForwardKinematics)(int wid, int skid);

#endif // #ifndef PYDART2_SKELETON_API_H
