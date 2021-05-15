#include <Arduino.h>

#include "boards/board_v_1_0.h"
#include "connections/starship_v_1_0.h"
#include "KraftKontrol.h"

#include "starship_specifics/starship.h"



SX1280Driver radio(SX1280_RFBUSY_PIN, SX1280_TXEN_PIN, SX1280_RXEN_PIN, SX1280_DIO1_PIN, SX1280_NRESET_PIN, SX1280_NSS_PIN);
KraftKommunication commsPort(&radio, eKraftPacketNodeID_t::eKraftPacketNodeID_vehicle);
UbloxSerialGNSS gnss(&NEO_M8Q_SERIALPORT);

BME280Driver Barometer(BME280_NCS_PIN, &BME280_SPIBUS);
MPU9250Driver IMU(MPU9250_INT_PIN, MPU9250_NCS_PIN, &MPU9250_SPIBUS);

NavigationComplementaryFilter navigationmodule(&IMU, &IMU, &IMU, &Barometer);
GuidanceFlyByWire guidanceModule;
StarshipControl controlModule(&guidanceModule, &navigationmodule);
StarshipDynamics dynamicsModule(&controlModule, &navigationmodule);

KraftKonnectNetwork network(&commsPort);

Starship vehicle(&guidanceModule, &navigationmodule, &controlModule, &dynamicsModule);



void radioControlPacketEventHandler() {


    KraftMessageRCChannels message;

    if (!commsPort.getMessage(&message)) return;

    int16_t channels[15];

    message.getChannelAll(channels);

    float x = (float)(channels[0]-1500)/500.0*15*DEGREES;
    float y = (float)(channels[1]-1500)/500.0*15*DEGREES;

    return;

    if (x != 0 && y != 0) {

        Vector rotVec = Vector(x, y, 0);

        Quaternion attitude = Quaternion(rotVec.copy().normalize(), rotVec.magnitude());

        guidanceModule.setAttitude(attitude);
        guidanceModule.setAngularRate(Vector(0,0,0));
        guidanceModule.setPosition(navigationmodule.getNavigationData().position);
        guidanceModule.setVelocity(Vector(0,0,0));

    }

}



void vehicleModeSetPacketEventHandler() {

    KraftMessageVehicleModeSet message;

    if (!commsPort.getMessage(&message)) return;

    eVehicleMode_t mode = message.getVehicleMode();
    
    if (mode == eVehicleMode_t::eVehicleMode_Arm) {
        vehicle.armVehicle();
    } else {
        vehicle.disarmVehicle();
    }

}



class Observer: public Task_Abstract {
public:

    Observer() : Task_Abstract(10, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        /*if (commsPort.getNodeStatus(eKraftPacketNodeID_t::eKraftPacketNodeID_controller) == false) {
            vehicle.disarmVehicle();
        }*/

        float angle = navigationmodule.getNavigationData().attitude.rotateVector(Vector(0,0,1)).getAngleTo(Vector(0,0,1));

        //Serial.println(navigationmodule.getNavigationData().position.z);
        //Serial.println(IMU.gyroRate());
        //Serial.println(String("speed: x: ") + navigationmodule.getNavigationData().velocity.x + ", y: " + navigationmodule.getNavigationData().velocity.x + ", z: " + navigationmodule.getNavigationData().velocity.z);
        //Serial.println(String("position: x: ") + navigationmodule.getNavigationData().position.x + ", y: " + navigationmodule.getNavigationData().position.x + ", z: " + navigationmodule.getNavigationData().position.z);
        Serial.println(String("GNSS status: ") + deviceStatusToString(gnss.getModuleStatus()) + String(", sats: ") + gnss.getNumSatellites() + " , rate: " + gnss.positionRate());

    }


};



class NetworkingTransmitter: public Task_Abstract {
public:

    NetworkingTransmitter() : Task_Abstract(10, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        //KraftMessageStringPacket stringPacket((String("Hello im ") + commsPort.getSelfID() + "! Time is: " + millis()).c_str());

        //commsPort.sendMessage(&stringPacket, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        //Serial.println("Sending: " + String("Hello im ") + commsPort.getSelfID() + "! Time is: " + millis());

        //KraftMessageFullKinematics message(navigationmodule.getNavigationData());

        //commsPort.sendMessage(&message, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        KraftMessageAttitude attitudeMessage(navigationmodule.getNavigationData().attitude);

        commsPort.sendMessage(&attitudeMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        KraftMessagePosition positionMessage(navigationmodule.getNavigationData().position, navigationmodule.getNavigationData().timestamp);

        //commsPort.sendMessage(&positionMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        if (gnss.positionAvailable()) {

            WorldPosition worldPosition; uint32_t time;
            gnss.getPosition(&worldPosition, &time);

            KraftMessageGNSSData gnssMessage(worldPosition, gnss.getNumSatellites());

            commsPort.sendMessage(&gnssMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        }

        

        KraftMessageVehicleModeIs vehicleMode(vehicle.getVehicleData().vehicleMode);

        commsPort.sendMessage(&vehicleMode, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        //stopTaskThreading();

    }


};


Observer observer;
NetworkingTransmitter transmitter;



void setup() {

    Serial.begin(115200);

    //Serial5.begin(115200);

    Task_Abstract::schedulerInitTasks();

    //User code here:
    network.setEventHandler(radioControlPacketEventHandler, eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_RCChannels);
    network.setEventHandler(vehicleModeSetPacketEventHandler, eKraftMessageType_KraftKontrol_t::eKraftMessageType_KraftKontrol_VehicleModeSet);

}


void loop() {

    Task_Abstract::schedulerTick();

    /*if (millis() > 6000 && vehicle.getVehicleData().vehicleMode != eVehicleMode_t::eVehicleMode_Arm) {
        vehicle.armVehicle();
        //dynamicsModule.setActuatorTesting();
    }*/
    
}


void idleLoop() {

    //Do unimportant stuff. Will not run if whole system is too busy.

}


void vehicleProgram() {

    /*runner.armVehicle(); //Arms vehicle. Must be called before giving commands. This readies the vehicle.

    runner.vehicleBusy(); //Returns false if vehicle is ready for next command. Calling a command when vehicle isnt ready will make vehicle ignore old command and start with new command

    runner.p2pRelative(Vector(2,2,1), VEHICLE_WAIT, MAX_SPEED, MAX_ACCEL) //Reach at max this speed and accel. Cannot be higher than vehicles built in limits. Relative to current SETPOINT
    runner.p2pRelative(Vector(2,2,1), VEHICLE_WAIT, DTIME) //Reach at earliest this time
    runner.p2pRelative(Vector(2,2,1), VEHICLE_CONTINUE, DTIME) //VEHICLE_CONTINUE parameter will make vehicle follow this command but this will immediately return giving user power to do what they want in the mean time. Use runner.vehicleBusy() to find out if vehicle has finished command. runner.tick() MUST be called as fast as possible.

    runner.p2pAbsolute(Vector(2,2,1), VEHICLE_WAIT, same as relative) //simply go to point. Extra settings or like relative

    runner.p2pLinearRelative() //Moves in a straight line. Settings like non linear
    runner.p2pLinearRelative(Vector(Start), Vector(end)) //Same as p2pLinearRelative(Vector(end)) but will use the parameter start as starting line.

    runner.circleRelative(Vector(2,2,1), radius, forTime, ) //radius is radius of circle. forTime is the lenght of time. Use VEHICLE_CONTINUE for indefinate circle time and call runner.tick() as fast as possible.
    runner.circleRelative(Vector(2,2,1), Vector(startpoint), forradians) //startpoint is the point at which the circle starts. forradians is the angle the circle will go for. Use VEHICLE_CONTINUE for indefinate circle time and call runner.tick() as fast as possible.

    vehicle.currentState() //returns current kinematic state (position, velocity, accel, attitude, angularRate, angularAccel)
    vehicle.currentStateSetpoint() //returns current kinematic state setpoint. Same as currentState() but returns the setpoint. Usefull for some commands.

    //Some examples for lower level control:

    guidance.setSetpoints(kinematics) //vehicle modules can also be used for stuff like manual flight.

    float manualSettings[4];
    radio.getManualControl(manualSettings);
    dynamics = manualSettings //use manual settings for manual control and produce some dynamic mapping for full manual control.
    control.setDynamics(dynamics); //give vehicle controlModules manual dynamics and then call runner.tick() to have full manual control over vehicle. Same can be done with control module to have control module help reach kinematic setpoints like angular rate etc.



    runner.progEnd() //Will wait and keep running vehicle indefinetely. Will disarm vehicle and stop running this vehicleprogram loop.
    */

}