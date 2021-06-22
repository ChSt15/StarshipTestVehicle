#ifndef STARSHIP_DYNAMICS_H
#define STARSHIP_DYNAMICS_H



#include "Arduino.h"

#include "lib/Simple-Schedule/src/task_autorun_class.h"

#include "outputs/servo_ppm.h"

#include "dynamics/servo_dynamics.h"
#include "dynamics/tvc_dynamics.h"

#include "modules/dynamics_modules/dynamics_interface.h"
#include "modules/navigation_modules/navigation_interface.h"
#include "modules/control_modules/control_interface.h"
#include "modules/module_abstract.h"

#include "data_containers/navigation_data.h"
#include "data_containers/dynamic_data.h"

#include "starship_connections_v_1_0.h"


//Flap servo accel and velocity maximums
#define FLAP_MAX_VELOCITY 5
#define FLAP_MAX_ACCEL 30

#define FLAP_START_MAX_ACCEL 5
#define FLAP_START_MAX_VELOCITY 1

//TVC constraints
#define MAX_TVC_ANGLE 30*DEGREES
#define MAX_TVC_FORCE 20.0f //In newtons

//Actuator mapping for manual control
#define STARSHIP_ACTUATORFLAPUL 0
#define STARSHIP_ACTUATORFLAPUR 1
#define STARSHIP_ACTUATORFLAPDR 2
#define STARSHIP_ACTUATORFLAPDL 3
#define STARSHIP_ACTUATOR_TVC1 4
#define STARSHIP_ACTUATOR_TVC2 5
#define STARSHIP_ACTUATOR_TVC3 6
#define STARSHIP_ACTUATOR_TVC4 7
#define STARSHIP_ACTUATORMOTORCW 8
#define STARSHIP_ACTUATORMOTORCCW 9

//Starship measurements
//Top flaps distance from CG in z direction
#define TOP_FLAP_Z_FROM_CG 0.5
//Bottom flaps distance from CG in z direction.
#define BOTTOM_FLAP_Z_FROM_CG 0.3
//Flap distance from CG to side
#define FLAP_XY_FROM_CG 0.1
//Area of top flaps
#define TOP_FLAP_AREA 7.69e-3
//Area of bottom flaps
#define BOTTOM_FLAP_AREA 12.15e-3



class StarshipDynamics: public Dynamics_Interface, public Module_Abstract, public Task_Abstract {
public:

    /**
     * Creates a module based class and automatically adds it to the scheduler.
     * 
     * @param controlModule is the module from which control data will be received.
     * @param navigationModule is the module from which navigation data will be received.
     */
    StarshipDynamics(Control_Interface* controlModule, Navigation_Interface* navigationModule) : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_High, true) {
        controlModule_ = controlModule;
        navigationModule_ = navigationModule;
        navigationData_ = navigationModule->getNavigationDataPointer();
    }

    /**
     * This is where all calculations are done.
     *
     */
    void thread();

    /**
     * Init function that sets the module up.
     *
     */
    void init();

    /**
     * Returns the forces the acuators need to produce in total
     * on the vehicle in order to achieve the kinematic setpoints. 
     *
     * @return DynamicData.
     */
    DynamicData getDynamicSetpoint() {return controlModule_->getDynamicsOutput();}

    /**
     * Returns a pointer towards a struct containing the forces 
     * the acuators need to produce in total on the vehicle in 
     * order to achieve the kinematic setpoints. 
     * 
     * Using pointers allows for data linking.
     *
     * @return DynamicData pointer.
     */
    DynamicData* getDynamicSetpointPointer() {return controlModule_->getDynamicsOutputPointer();}

    /**
     * Tells dynamics module to test all actuators. 
     * Giving false will stop testing.
     *
     * @param testing Default is true
     */
    void setActuatorTesting(bool testing = true) {
        actuatorTesting_ = testing;
    }

    /**
     * Returns if actuators are being tested.
     *
     * @returns boolean.
     */
    bool getActuatorTesting() {
        return actuatorTesting_;
    }

    /**
     * Tells dynamics module to enter manual mode
     * Giving false will stop testing.
     *
     * @param values bool.
     */
    void setActuatorManualMode(bool testing = true) {
        actuatorManualMode_ = testing;
    }

    /**
     * Returns if currently in actuator tesing mode.
     *
     * @returns boolean.
     */
    bool getActuatorManualMode() {
        return actuatorManualMode_ = false;
    }

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @param actuatorSetpoint Actuator Setting.
     * @return none.
     */
    void setActuatorsRawData(const ActuatorSetting &actuatorSetpoint) {
        actuatorManualSetpoint_ = actuatorSetpoint;
    }

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @param actuatorSetpoint Actuator Setting.
     * @param actuatorNum actuator to change.
     * @return none.
     */
    void setActuatorsRawData(const float &actuatorSetpoint, const uint16_t &actuatorNum) {
        actuatorManualSetpoint_.actuatorSetting[actuatorNum] = actuatorSetpoint;
    }

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @returns actuator setting.
     */
    ActuatorSetting getActuatorsRawData() {return actuatorManualSetpoint_;}

    /**
     * Used to send raw actuator commands to actuators.
     * Only valid if in actuator manual mode. Call _enterActuatorManualMode()
     * to enter this mode.
     *
     * @param actuatorNum actuator to get.
     * @returns actuator setting.
     */
    float getActuatorsRawData(const uint16_t &actuatorNum) {return actuatorManualSetpoint_.actuatorSetting[actuatorNum];}

    /**
     * Sets the actuator status. 
     * @param actuatorStatus Actuator mode to set to.
     */
    void setActuatorStatus(const eActuatorStatus_t &actuatorStatus) {actuatorStatusSetpoint_ = actuatorStatus;}

    /**
     * Checks if module is ready. Should only return true if all actuators are in position and ready to follow commands.
     */
    eActuatorStatus_t getActuatorStatus() {return actuatorStatus_;}

    /**
     * Tells dynamics to use flaps.
     * @param enable True to use flaps. Defaults to true.
     */
    void enableFlaps(const bool &enable = true) {enableFlaps_ = enable;}

    /**
     * Tells dynamics to use TVC.
     * @param enable True to use TVC. Defaults to true.
     */
    void enableTVC(const bool &enable = true) {enableTVC_ = enable;}
    

private:

    //Whether the vehicle has been initialised
    bool initialised_ = false;

    //Enum for actuator status
    eActuatorStatus_t actuatorStatus_ = eActuatorStatus_t::eActuatorStatus_Disabled;
    eActuatorStatus_t actuatorStatusLast_ = eActuatorStatus_t::eActuatorStatus_Disabled;
    eActuatorStatus_t actuatorStatusSetpoint_ = eActuatorStatus_t::eActuatorStatus_Disabled;

    //Points to dynamics setpoint data container.
    Control_Interface* controlModule_;

    //Points to navigation data container.
    Navigation_Interface* navigationModule_;
    NavigationData* navigationData_;

    //When true then start tesing all actuators. Can be set to true by module when testing is finished.
    bool actuatorTesting_ = false;

    //When true then ignore dynamic setpoints and only use manual setpoints
    bool actuatorManualMode_ = false;

    //Contains actuator manual raw setpoints. Should only be used when in manual mode.
    ActuatorSetting actuatorManualSetpoint_;

    //If true then disable motors and TVC and use flaps
    bool enableFlaps_ = false;
    bool enableTVC_ = true;

    //TVC servo in X+
    PPMChannel TVCServo1_ = PPMChannel(TVC_SERVO_PIN_1, ePPMProtocol_t::ePPMProtocol_Standard_1000us, 0, -1);
    //TVC servo in Y+
    PPMChannel TVCServo2_ = PPMChannel(TVC_SERVO_PIN_2, ePPMProtocol_t::ePPMProtocol_Standard_1000us, 0, -1);
    //TVC servo in X-
    PPMChannel TVCServo3_ = PPMChannel(TVC_SERVO_PIN_3, ePPMProtocol_t::ePPMProtocol_Standard_1000us, 0, 1);
    //TVC servo in Y-
    PPMChannel TVCServo4_ = PPMChannel(TVC_SERVO_PIN_4, ePPMProtocol_t::ePPMProtocol_Standard_1000us, -0.1, 1);
    //CW motor control.
    PPMChannel motorCW_ = PPMChannel(MOTOR_PIN_CW, ePPMProtocol_t::ePPMProtocol_Standard_1000us, -1, 2);
    //CCW motor control.
    PPMChannel motorCCW_ = PPMChannel(MOTOR_PIN_CCW, ePPMProtocol_t::ePPMProtocol_Standard_1000us, -1, 2);
    //Up Left flap servo.
    PPMChannel flapUL_ = PPMChannel(FLAP_SERVO_PIN_UL, ePPMProtocol_t::ePPMProtocol_Standard_1000us, 1, -1);
    ServoDynamics flapULControl_ = ServoDynamics(&flapUL_, 1*DEGREES, FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);
    //Up Right flap servo.
    PPMChannel flapUR_ = PPMChannel(FLAP_SERVO_PIN_UR, ePPMProtocol_t::ePPMProtocol_Standard_1000us, -1, 1);
    ServoDynamics flapURControl_ = ServoDynamics(&flapUR_, -5*DEGREES, FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);
    //Down Left flap servo.
    PPMChannel flapDL_ = PPMChannel(FLAP_SERVO_PIN_DL, ePPMProtocol_t::ePPMProtocol_Standard_1000us, -1, 1);
    ServoDynamics flapDLControl_ = ServoDynamics(&flapDL_, -7*DEGREES, FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);
    //Down Right flap servo.
    PPMChannel flapDR_ = PPMChannel(FLAP_SERVO_PIN_DR, ePPMProtocol_t::ePPMProtocol_Standard_1000us, 1, -1);
    ServoDynamics flapDRControl_ = ServoDynamics(&flapDR_, 1*DEGREES, FLAP_MAX_VELOCITY, FLAP_MAX_ACCEL);

    //Enum for flap testing
    enum FLAP_TEST_STAGE {
        TOP_LEFT,
        TOP_RIGHT,
        BOTTOM_LEFT,
        BOTTOM_RIGHT
    };

    FLAP_TEST_STAGE flapTestStage_ = FLAP_TEST_STAGE::TOP_LEFT;
    uint8_t flapTestCounter_ = 0;

    //Helps with calculating TVC stuff
    TVCDynamics TVCCalculator_ = TVCDynamics(Vector<>(0,0,-0.4), Vector<>(0,0,-1));


    void getTVCAngles(const Vector<> &direction, const float &twist, float &tvc1, float &tvc2, float &tvc3, float &tvc4);

    DynamicData tvcDynamicsCalculation(const DynamicData &currentDynamics);

    DynamicData flapsDynamicsCalculation(const DynamicData &currentDynamics);

    //Used for calculating optimal angle of flaps.
    float getAngle(const float &fx, const float &fy, const float &windSpeed, const float &airDensity, const float &dragCoefficient, const float &flapArea) {
        float fv = 0.5*airDensity*dragCoefficient*windSpeed*windSpeed*flapArea;
        if (fv < 0.00001f) return 90*DEGREES;
        float fyTheta = 1.13f-fy/fv;
        float fxTheta = -fx/fv + 0.38;
        if (fxTheta >= 0) fxTheta = sqrt(abs(fxTheta)) + PI/5;
        else fxTheta = -sqrt(abs(fxTheta)) + PI/5;
        return fxTheta;
    }
    
};





#endif