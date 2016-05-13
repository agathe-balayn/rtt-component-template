#include <rtt/Component.hpp>


#include "rtt-jpc.hpp"


RTTJointPositionController::RTTJointPositionController(std::string const& name) : RTT::TaskContext(name), currJntPos_Flow(RTT::NoData) {
    this->addPort("cmdJntTrq", cmdJntTrq_Port);
    trqCmdOutput = rci::JointTorques::create(7, 0.0);
    cmdJntTrq_Port.setDataSample(trqCmdOutput);

    this->addPort("currJntPos", currJntPos_Port);
    this->addPort("currJntVel", currJntVel_Port);
    this->addPort("refJntPos", refJntPos_Port);
    this->addPort("refJntVel", refJntVel_Port).doc("Reference Joint Velocity Port");


}

bool RTTJointPositionController::configureHook() {
    if (!(cmdJntTrq_Port.connected() && currJntPos_Port.connected() && currJntVel_Port.connected() && refJntPos_Port.connected() && refJntVel_Port.connected())) {
        return false;
    }
    return true;
}

bool RTTJointPositionController::startHook() {
    return true;
}

void RTTJointPositionController::updateHook() {
    currJntPos_Flow = currJntPos_Port.read(currPositionFB);
    if (currJntPos_Flow == RTT::NewData) {

    }

    // calculations
    //trqCmdOutput =


    if (cmdJntTrq_Port.connected()) {
        cmdJntTrq_Port.write(trqCmdOutput);
    }
}

void RTTJointPositionController::stopHook() {

}

void RTTJointPositionController::cleanupHook() {

}


ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(RTTJointPositionController)
