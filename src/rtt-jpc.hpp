#ifndef RTT_JPC_HPP
#define RTT_JPC_HPP
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
#include "parsertools/KDLParser.hpp"

class RTTJointPositionController: public RTT::TaskContext {
public:
    RTTJointPositionController(std::string const& name);
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();


    RTT::OutputPort<rci::JointTorquesPtr> cmdJntTrq_Port;


    RTT::InputPort<rci::JointAnglesPtr> currJntPos_Port;
    RTT::FlowStatus currJntPos_Flow;

    RTT::InputPort<rci::JointVelocitiesPtr> currJntVel_Port;
    RTT::FlowStatus currJntVel_Flow;


    RTT::InputPort<rci::JointAnglesPtr> refJntPos_Port;
    RTT::FlowStatus refJntPos_Flow;

    RTT::InputPort<rci::JointVelocitiesPtr> refJntVel_Port;
    RTT::FlowStatus refJntVel_Flow;

    rci::JointTorquesPtr trqCmdOutput;
    rci::JointAnglesPtr currPositionFB;

};
#endif
