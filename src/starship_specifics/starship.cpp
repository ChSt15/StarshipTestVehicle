#include "starship.h"



void Starship::thread() {

    if (!vehicleInitialized_) init();

    NavigationData navData = navigation_->getNavigationData();

    //Handle mode changes
    switch (vehicleData_.vehicleMode) {

    case eVehicleMode_t::eVehicleMode_Arm:

        switch (dynamics_->getActuatorStatus()) {
            
        case eActuatorStatus_t::eActuatorStatus_Disabled:
            dynamics_->setActuatorStatus(eActuatorStatus_t::eActuatorStatus_Ready);
            break;

        case eActuatorStatus_t::eActuatorStatus_Ready:
            dynamics_->setActuatorStatus(eActuatorStatus_t::eActuatorStatus_Enabled);
            break;

        default:
            break;

        }

        break;

    case eVehicleMode_t::eVehicleMode_Prepare:
    
        if (/*navData.linearAcceleration.magnitude() < 0.5 && navData.velocity.magnitude() < 0.5*/ true ) {
            navigation_->setHome(navData.absolutePosition);
            //if (dynamics_->getActuatorStatus() == eActuatorStatus_t::eActuatorStatus_Disabled) dynamics_->setActuatorStatus(eActuatorStatus_t::eActuatorStatus_Ready);
            vehicleData_.vehicleMode = eVehicleMode_t::eVehicleMode_Arm;
        }
        break;

    default:
        dynamics_->setActuatorStatus(eActuatorStatus_t::eActuatorStatus_Disabled);
        break;

    }

}


void Starship::init() {

    //Setup hover controller
    //control_->setAngularVelocityPIDFactors(Vector(1,1,0.1), Vector(0), Vector(0), Vector(1000), true);
    control_->setAttitudePIDFactors(Vector(10,10,5), Vector(0), Vector(0), Vector(1000), true);
    control_->setPositionPIDFactors(Vector(0,0,20), Vector(0), Vector(0), Vector(1000), true);

    //Mark that vehicle has been initialized.
    vehicleInitialized_ = true;

}