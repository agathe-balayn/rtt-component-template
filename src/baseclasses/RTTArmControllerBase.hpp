#ifndef RTT_ARM_CONTROLLER_BASE_HPP
#define RTT_ARM_CONTROLLER_BASE_HPP
// RTT includes
#include <rtt/RTT.hpp>
#include <rtt/base/RunnableInterface.hpp>
#include <rtt/Activity.hpp>
// RCI includes
#include <rci/dto/JointAngles.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointTorques.h>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
// Parser include convert URDF/SDF into KDL::Chain
#include "../parsertools/KDLParser.hpp"

/**
 *
 * RTTArmControllerBase is used as base-class to wrap
 * the creation of the KDL::Chain based on an URDF/SDF
 * and the update of the dynamic model properties.
 *
 */
class RTTArmControllerBase: public RTT::TaskContext {
public:
	/**
	 * This constructor collects the necessary data for the KDL::Chain creation.
	 *
	 * @name name of the RTT::TaskContext.
	 * @default_root_link name of the default root link in the URDF/SDF (e.g. lwr_arm_base_link).
	 * @default_tip_link name of the default tip link in the URDF/SDF (e.g. lwr_arm_7_link).
	 * @default_nr_joints total amount of joints (e.g. 7).
	 *
	 */
	RTTArmControllerBase(std::string const& name, std::string default_root_link,
			std::string default_tip_link, int default_nr_joints);
	/**
	 * It is accessible from other RTT::TaskContext by the RTT environment.
	 * Its purpose is to create the KDL::Chain based on the parameters provided by the URDF/SDF,
	 * which is coming from e.g. the Gazebo simulation.
	 *
	 * @urdfString URDF/SDF string.
	 *
	 * @returns indicates if the parsing went well or not.
	 */
	bool parseURDFforKDL(std::string urdfString);

protected:
	/**
	 * Updates the dynamic model properties and makes them available to be used in
	 * e.g. the update loop of the controller.
	 *
	 * Global variables that are updated:
	 * -> M_
	 * -> G_
	 * -> C_
	 * -> jac_
	 *
	 * @jPosFb current joint positions
	 * @jVelFb current joint velocities
	 * @jTrqFb current joint torques
	 *
	 * @returns whether the update went well or not.
	 */
	bool updateDynamicsAndKinematics(rci::JointAnglesPtr jPosFb,
			rci::JointVelocitiesPtr jVelFb, rci::JointTorquesPtr jTrqFb);
	/**
	 * Initializes the KDL storage variables.
	 */
	void initKDLTools();

	// variables used for URDF/SDF to KDL::Chain parsing
	std::string DEFAULT_ROOT_LINK;
	std::string DEFAULT_TIP_LINK;
	int DEFAULT_NR_JOINTS;

	// container for URDF string
	std::string urdfContainer;
	// name of the root link for the KDL chain
	std::string chain_root_link_name;
	// name of the tip link for the KDL chain
	std::string chain_tip_link_name;
	// KDL::Tree needed for KDL::Chain creation
	KDL::Tree kdl_tree_;
	// Needed for the different solvers
	KDL::Chain kdl_chain_;
	// Contains the gravity information about the environment
	KDL::Vector gravity_vector;
	KDL::JntArrayVel jntPosConfigPlusJntVelConfig_q;
	KDL::JntSpaceInertiaMatrix M_;
	KDL::JntArray G_;
	KDL::JntArray C_;
	KDL::Jacobian jac_;
	KDL::Jacobian jac_dot_;
	// Helper tools for KDL
	KDLParser p;
	// for conversion to use KDL
	Eigen::VectorXd jnt_pos_, jnt_vel_, jnt_trq_;
	Eigen::VectorXd grav_trq_;
	// KDL solver
	boost::scoped_ptr<KDL::ChainDynParam> id_dyn_solver;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver;

	boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;
	boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_cart_vel_solver;

	KDL::Frame cartFrame;
	KDL::FrameVel velFrame;


	// status for preconditions
	bool properlyParsed;
	bool kdlFieldsInitialized;

    //Pouya & MIlad:
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> rne_solver;
};
#endif
