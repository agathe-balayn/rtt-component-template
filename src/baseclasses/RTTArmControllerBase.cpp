#include "RTTArmControllerBase.hpp"

#include <rtt/Component.hpp>
#include <iostream>
#include <rtt/Activity.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;
using namespace rci;

bool RTTArmControllerBase::updateDynamicsAndKinematics(
		rci::JointAnglesPtr jPosFb, rci::JointVelocitiesPtr jVelFb,
		rci::JointTorquesPtr jTrqFb) {

	if (!kdlFieldsInitialized || !properlyParsed) {
		return false;
	}

	/* ### Convert Data to be used with KDL solvers */
	p.convertRealVectorToEigenVectorXd(jPosFb->radVector(), jnt_pos_);
	p.convertRealVectorToEigenVectorXd(jVelFb->rad_sVector(), jnt_vel_);
	p.convertRealVectorToEigenVectorXd(jTrqFb->NmVector(), jnt_trq_);

	/* ### initialize strange stuff for solvers */
	jntPosConfigPlusJntVelConfig_q.q.data = jnt_pos_;
	jntPosConfigPlusJntVelConfig_q.qdot.data = jnt_vel_;


	/* ### execute solvers for inv.Dynamics */
	// calculate matrices H (inertia),C(coriolis) and G(gravitation)
	id_dyn_solver->JntToMass(jntPosConfigPlusJntVelConfig_q.q, M_);

	id_dyn_solver->JntToGravity(jntPosConfigPlusJntVelConfig_q.q, G_);
	id_dyn_solver->JntToCoriolis(jntPosConfigPlusJntVelConfig_q.q, jntPosConfigPlusJntVelConfig_q.qdot, C_);

	/* ### execute solver for Jacobian based on velocities */
	jnt_to_jac_solver->JntToJac(jntPosConfigPlusJntVelConfig_q.q, jac_, kdl_chain_.getNrOfSegments());

	jnt_to_jac_dot_solver->JntToJacDot(jntPosConfigPlusJntVelConfig_q, jac_dot_, kdl_chain_.getNrOfSegments());

	// jnt to cart pos
	jnt_to_cart_pos_solver->JntToCart(jntPosConfigPlusJntVelConfig_q.q, cartFrame, kdl_chain_.getNrOfSegments());
	jnt_to_cart_vel_solver->JntToCart(jntPosConfigPlusJntVelConfig_q, velFrame, kdl_chain_.getNrOfSegments());


//	log(Error) << "jac_dot_\n" << jac_dot_ << endlog();
//	l(Error) << "H: \n" << H << endlog();
//	l(Error) << "G: \n" << G << endlog();
//	l(Error) << "C: \n" << jnt_trq_coriolis_kdl_ << endlog();
}

void RTTArmControllerBase::initKDLTools() {
	// initialize KDL fields
	jntPosConfigPlusJntVelConfig_q.resize(DEFAULT_NR_JOINTS);
	G_.resize(DEFAULT_NR_JOINTS);
	jac_.resize(DEFAULT_NR_JOINTS);
	jac_dot_.resize(DEFAULT_NR_JOINTS);
	jac_dot_.data.setZero();
	jac_.data.setZero();
	M_.resize(DEFAULT_NR_JOINTS);
	jnt_pos_.resize(DEFAULT_NR_JOINTS);
	jnt_pos_.setZero();
	jnt_vel_.resize(DEFAULT_NR_JOINTS);
	jnt_vel_.setZero();
	C_.resize(DEFAULT_NR_JOINTS);
	jnt_trq_.resize(DEFAULT_NR_JOINTS);
	jnt_trq_.setZero();
	grav_trq_.resize(DEFAULT_NR_JOINTS);
	grav_trq_.setZero();

	kdlFieldsInitialized = true;
}

bool RTTArmControllerBase::parseURDFforKDL(string urdfString) {
	urdfContainer = urdfString; // perhaps a mutex here TODO

	if (!p.initTreeAndChainFromURDFString(urdfString, chain_root_link_name,
			chain_tip_link_name, kdl_tree_, kdl_chain_)) {
		log(Error) << "[" << this->getName() << "] URDF could not be parsed !"
				<< endlog();
		return false;
	}

	if (kdl_tree_.getNrOfJoints() > 0) {
		log(Info) << "URDF parsed !" << endlog();

		log(Info) << "NrOfJoints: " << kdl_tree_.getNrOfJoints()
				<< ", NrOfSegments: " << kdl_tree_.getNrOfSegments()
				<< endlog();

		SegmentMap::const_iterator it;
		for (it = kdl_tree_.getSegments().begin();
				it != kdl_tree_.getSegments().end(); it++) {
			log(Info) << "Parsed Joints in KDL-Tree " << it->first << endlog();
		}
		// initialize solvers
		id_dyn_solver.reset(new ChainDynParam(kdl_chain_, gravity_vector));
		jnt_to_jac_solver.reset(new ChainJntToJacSolver(kdl_chain_));
		jnt_to_jac_dot_solver.reset(new ChainJntToJacDotSolver(kdl_chain_));

		jnt_to_cart_pos_solver.reset(new ChainFkSolverPos_recursive(kdl_chain_));
		jnt_to_cart_vel_solver.reset(new ChainFkSolverVel_recursive(kdl_chain_));

        //Pouya:
        rne_solver.reset(new ChainIdSolver_RNE(kdl_chain_, gravity_vector));

//		log(Error) << "[RTTArmControllerBase] kdl_chain_:\n" << kdl_chain_ << endlog();
		properlyParsed = true;
		return true;
	} else {
		log(Error) << "[" << this->getName() << "] URDF could not be parsed !"
				<< endlog();
		return false;
	}
}

RTTArmControllerBase::RTTArmControllerBase(std::string const& name,
		std::string default_root_link, std::string default_tip_link,
		int default_nr_joints) :
		TaskContext(name), DEFAULT_ROOT_LINK(default_root_link), DEFAULT_TIP_LINK(
				default_tip_link), DEFAULT_NR_JOINTS(default_nr_joints), chain_root_link_name(
				DEFAULT_ROOT_LINK), chain_tip_link_name(DEFAULT_TIP_LINK), gravity_vector(
				0., 0., -9.81289), properlyParsed(false), kdlFieldsInitialized(
				false) {
	this->addOperation("parseURDFforKDL",
			&RTTArmControllerBase::parseURDFforKDL, this, OwnThread).doc(
			"Parses a URDF string to create a KDL::Tree.").arg("urdfString",
			"URDF string to parse.");

	initKDLTools();
}
