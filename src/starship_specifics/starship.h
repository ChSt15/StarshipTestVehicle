#ifndef STARSHIP_H
#define STARSHIP_H



#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "data_containers/navigation_data.h"

#include "vehicle/vehicle_interface.h"

#include "modules/navigation_modules/navigation_complementary.h"
#include "modules/guidance_modules/guidance_flybywire.h"

#include "starship_dynamics.h"
#include "starship_control.h"



class Starship: public Vehicle_Interface, Task_Abstract {
public:

    Starship(Guidance_Interface* guidancePointer, Navigation_Interface* navigationPointer, StarshipControl* controlPointer, StarshipDynamics* dynamicsPointer) : Task_Abstract(8000, eTaskPriority_t::eTaskPriority_High, true) {
        guidance_ = guidancePointer;
        navigation_ = navigationPointer;
        control_ = controlPointer;
        dynamics_ = dynamicsPointer;
    }

    /**
     * Thread function of the vehicle. 
     * All calculations the vehicle ever has to do for its 
     * control will be done here.
     *
     * @param values none.
     * @return none.
     */
    void thread();

    /**
     * Init function that setups the vehicle. If not called
     * then on the first Thread run this will automatically 
     * be called.
     *
     * @param values none.
     * @return none.
     */
    void init();

    /**
     * Returns pointer to the navigation module the vehicle uses. 
     * @returns Navigation_Interface
     */
    Navigation_Interface* getNavigationModulePointer() {return navigation_;}

    /**
     * Returns pointer to the guidance module the vehicle uses. 
     * @returns Guidance_Interface
     */
    Guidance_Interface* getGuidanceModulePointer() {return guidance_;}

    /**
     * Returns pointer to the control module the vehicle uses. 
     * @returns Control_Interface
     */
    Control_Interface* getControlModulePointer() {return control_;}

    /**
     * Returns pointer to the dynamics module the vehicle uses. 
     * @returns Dynamics_Interface
     */
    Dynamics_Interface* getDynamicsModulePointer() {return dynamics_;}

    /**
     * Arms vehicle. Vehicle will ready itsself and then go into flight mode.
     */
    void armVehicle() {
        if(vehicleData_.vehicleMode != eVehicleMode_t::eVehicleMode_Error && vehicleData_.vehicleMode != eVehicleMode_t::eVehicleMode_Arm) vehicleData_.vehicleMode = eVehicleMode_t::eVehicleMode_Prepare;
    }

    /**
     * Disarms vehicle. Vehicle should shutdown immediately
     */
    void disarmVehicle() {

        if(vehicleData_.vehicleMode != eVehicleMode_t::eVehicleMode_Error) {vehicleData_.vehicleMode = eVehicleMode_t::eVehicleMode_Disarm;}
        
    }

    /**
     * Gets the vehicle data.
     * 
     * @returns VehicleData of vehicle
     */
    VehicleData getVehicleData() {return vehicleData_;}


private:

    bool vehicleInitialized_ = false;

    //Contains vehicle data.
    VehicleData vehicleData_;

    //Points to the navigation module to use.
    Navigation_Interface* navigation_;

    //Points to the guidance module to use.
    Guidance_Interface* guidance_;

    //Points to the control module to use.
    StarshipControl* control_;

    //Points to the dynamics module to use.
    StarshipDynamics* dynamics_;


};




#endif