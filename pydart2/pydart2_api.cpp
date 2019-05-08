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
#include "pydart2_world_api.h"
#include "pydart2_skeleton_api.h"
#include "pydart2_bodynode_api.h"
#include "pydart2_api.h"


#include "pydart2_draw.h"

#include "NonHolonomicContactConstraint.h"

using namespace pydart;

////////////////////////////////////////////////////////////////////////////////
// Init Functions
void init(bool verbose) {
    setVerbose(verbose);
    if (Manager::getInstance()) {
        Manager::destroy();
    }
    Manager::init();
}

void destroy() {
    Manager::destroy();
}

void setVerbose(bool verbose) {
    Manager::g_verbose = verbose;
}

bool getVerbose() {
    return Manager::g_verbose;
}

////////////////////////////////////////////////////////////////////////////////
// Marker

int MARKER(getBodyNode)(int wid, int skid, int mid) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    return marker->getBodyNodePtr()->getIndexInSkeleton();
}


void MARKER(getLocalPosition)(int wid, int skid, int mid, double outv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    write(marker->getLocalPosition(), outv3);
}


void MARKER(setLocalPosition)(int wid, int skid, int mid, double inv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    marker->setLocalPosition(read(inv3, 3));
}


void MARKER(getWorldPosition)(int wid, int skid, int mid, double outv3[3]) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    write(marker->getWorldPosition(), outv3);
}

void MARKER(render)(int wid, int skid, int mid) {
    dart::dynamics::Marker* marker = GET_MARKER(wid, skid, mid);
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawMarker(ri, marker);
}

////////////////////////////////////////////////////////////////////////////////
// Collision Result
int COLLISION_RESULT(getNumContacts)(int wid) {
    const auto result = GET_COLLISION_RESULT(wid);
    return result.getNumContacts();
}

void COLLISION_RESULT(getContacts)(int wid, double* outv, int nout) {
    const auto result = GET_COLLISION_RESULT(wid);
    const auto nContacts = static_cast<int>(result.getNumContacts());

    // Construct the skeleton index map
    std::map<const dart::dynamics::Skeleton*, int> indices;
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    for (size_t i = 0; i < world->getNumSkeletons(); ++i) {
        indices[world->getSkeleton(i).get()] = i;
    }

    Eigen::VectorXd state(12 * nContacts);
    for (auto i = 0; i < nContacts; ++i) {
        auto begin = i * 12;
        auto contact = result.getContact(i);

        state.segment(begin, 3)     = contact.point;
        state.segment(begin + 3, 3) = contact.force;
        state(begin + 6) = -1;
        state(begin + 7) = -1;
        state(begin + 8) = -1;
        state(begin + 9) = -1;
        state(begin + 10) = -1;
        state(begin + 11) = -1;

        auto shapeNode1 = contact.collisionObject1->getShapeFrame()->asShapeNode();
        if (shapeNode1) {
            auto b = shapeNode1->getBodyNodePtr();
            state(begin + 6) = indices[b->getSkeleton().get()];
            state(begin + 7) = b->getIndexInSkeleton();
            state(begin + 8) = shapeNode1->getIndexInBodyNode();
        }

        auto shapeNode2 = contact.collisionObject2->getShapeFrame()->asShapeNode();
        if (shapeNode2) {
            auto b = shapeNode2->getBodyNodePtr();
            state(begin + 9) = indices[b->getSkeleton().get()];
            state(begin + 10) = b->getIndexInSkeleton();
            state(begin + 11) = shapeNode2->getIndexInBodyNode();
        }
    }
    write(state, outv);
}

std::vector<int> COLLISION_RESULT(getCollidingBodyNodes)(int wid) {
    const auto result = GET_COLLISION_RESULT(wid);
    const auto bodynodes = result.getCollidingBodyNodes();
    std::vector<int> ret;
    std::map<const dart::dynamics::Skeleton*, int> indices;
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    for (size_t i = 0; i < world->getNumSkeletons(); ++i) {
        indices[world->getSkeleton(i).get()] = i;
    }

    for (auto b: bodynodes) {
        ret.push_back(indices[b->getSkeleton().get()]);
        ret.push_back(b->getIndexInSkeleton());
    }
    return ret;
}

void COLLISION_RESULT(renderContact)(double inv6[6], double size, double scale) {
    dart::gui::RenderInterface* ri = Manager::getRI();
    drawContact(ri, read(inv6, 6), size, scale);
}

////////////////////////////////////////////////////////////////////////////////
// Constraints
int addBallJointConstraint(int wid, int skid1, int bid1, int skid2, int bid2,
                           double inv3[3]) {
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    dart::dynamics::BodyNodePtr bd1 = GET_BODY(wid, skid1, bid1);
    dart::dynamics::BodyNodePtr bd2 = GET_BODY(wid, skid2, bid2);
    Eigen::Vector3d jointPos = read(inv3, 3);
    // MSG << bd1->getName() << "\n";
    // MSG << bd2->getName() << "\n";
    // MSG << jointPos << "\n";
    dart::constraint::BallJointConstraintPtr cl =
        std::make_shared<dart::constraint::BallJointConstraint>(bd1, bd2, jointPos);
    world->getConstraintSolver()->addConstraint(cl);
    return 0;
}

int createNonHolonomicContactConstraint(int wid, int skid, int bid, double inv3[3])
{
    Eigen::Vector3d offset = read(inv3, 3);
    return Manager::createNHCConstraint(wid, skid, bid, offset);
}

int addNonHolonomicContactConstraint(int nhcid)
{
    int wid = Manager::nhconstraint_wid(nhcid);
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->getConstraintSolver()->addConstraint(Manager::nhconstraint(nhcid));

    return 0;
}

int setViolationAngleIgnoreThreshold(int nhcid, double th)
{
    dart::constraint::NonHolonomicContactConstraintPtr cl = Manager::nhconstraint(nhcid);
    cl->setViolationAngleIgnoreThreshold(th);
    return 0;
}

int setLengthForViolationIgnore(int nhcid, double length)
{
    dart::constraint::NonHolonomicContactConstraintPtr cl = Manager::nhconstraint(nhcid);
    cl->setLengthForViolationIgnore(length);
    return 0;
}


int setNonHolonomicContactConstraintActivate(int nhcid, bool activate)
{
    dart::constraint::NonHolonomicContactConstraintPtr cl = Manager::nhconstraint(nhcid);
    cl->activate(activate);
    return 0;
}

int setNonHolonomicContactConstraintJointPos(int nhcid, double inv3[3])
{
    dart::constraint::NonHolonomicContactConstraintPtr cl = Manager::nhconstraint(nhcid);
    Eigen::Vector3d jointPos = read(inv3, 3);
    cl->setJointPos(jointPos);
    return 0;
}

int setNonHolonomicContactConstraintProjectedVector(int nhcid, double inv3[3])
{
    dart::constraint::NonHolonomicContactConstraintPtr cl = Manager::nhconstraint(nhcid);
    Eigen::Vector3d projectedVector = read(inv3, 3);
    cl->setProjectedVector(projectedVector);
    return 0;
}


int createNonHolonomicContactConstraintV2(int wid, int skid, int bid, double inv3[3])
{
    Eigen::Vector3d offset = read(inv3, 3);
    return Manager::createNHCConstraintV2(wid, skid, bid, offset);
}

int addNonHolonomicContactConstraintV2(int nhcid)
{
    int wid = Manager::nhconstraint_widV2(nhcid);
    dart::simulation::WorldPtr world = GET_WORLD(wid);
    world->getConstraintSolver()->addConstraint(Manager::nhconstraintV2(nhcid));

    return 0;
}

int setViolationAngleIgnoreThresholdV2(int nhcid, double th)
{
    dart::constraint::NonHolonomicContactConstraintV2Ptr cl = Manager::nhconstraintV2(nhcid);
    cl->setViolationAngleIgnoreThreshold(th);
    return 0;
}

int setNonHolonomicContactConstraintActivateV2(int nhcid, bool activate)
{
    dart::constraint::NonHolonomicContactConstraintV2Ptr cl = Manager::nhconstraintV2(nhcid);
    cl->activate(activate);
    return 0;
}

