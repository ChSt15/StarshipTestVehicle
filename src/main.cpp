#include <Arduino.h>

#include "boards/board_v_1_0.h"
#include "connections/starship_v_1_0.h"
#include "KraftKontrol.h"

#include "vehicle/starship/starship.h"


SX1280Driver radio = SX1280Driver(SX1280_RFBUSY_PIN, SX1280_TXEN_PIN, SX1280_RXEN_PIN, SX1280_DIO1_PIN, SX1280_NRESET_PIN, SX1280_NSS_PIN);
KraftKommunication commsPort(&radio, eKraftPacketNodeID_t::eKraftPacketNodeID_vehicle);

BME280Driver Barometer = BME280Driver(BME280_NCS_PIN, &BME280_SPIBUS);
MPU9250Driver IMU = MPU9250Driver(MPU9250_INT_PIN, MPU9250_NCS_PIN, &MPU9250_SPIBUS);

NavigationComplementary navigationmodule(&IMU, &IMU, &IMU, &Barometer);
GuidanceFlyByWire guidanceModule;
HoverController controlModule(&guidanceModule, &navigationmodule);
StarshipDynamics dynamicsModule(&controlModule, &navigationmodule);

//Radio radioModule; //This modules takes care of communication with the goundstation and radiocontrol. It receives waypoints or also manual control commands from controllers.

Starship vehicle(&guidanceModule, &navigationmodule, &controlModule, &dynamicsModule);


void vehicleProgram(); //Has all functions needed for vehicle testing, waypoints and stuff
void idleLoop(); //Will be ran whenever free time is available. DO NOT BLOCK e.g with delay(). Things that NEED to be ran should be placed in normal loop().
//Runner runner(&vehicle, &vehicleProgram, &radioModule, &idleLoop)



class Observer: public Task_Abstract {
public:

    Observer() : Task_Abstract(20, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        NavigationData navData = navigationmodule.getNavigationData();

        Quaternion att = navData.attitude;
        Vector vec = navData.attitude.copy().conjugate().rotateVector(navData.angularAcceleration);
        float altitude = navData.position.z;

        //Serial.println(String("Attitude: w: ") + att.w + ", x: " + att.x + ", y: " + att.y + ", z: " + att.z + ". Altitude: " + altitude);
        //Serial.println(String("Vec: x: ") + vec.x + ", y: " + vec.y + ", z: " + vec.z);

        //Serial.println(String("Hello World! Speed: ") + F_CPU_ACTUAL + ", Rate: " + getSchedulerTickRate());

    }


};


uint32_t packetsCounter = 0;


class PacketsPerSecond: public Task_Abstract {
public:

    PacketsPerSecond() : Task_Abstract(1, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        float dTime = float(micros() - lastRun)/1000000;
        lastRun = micros();

        packetRate = packetsCounter/dTime;
        packetsCounter = 0;

    }

    uint32_t packetRate = 0;

    uint32_t lastRun = 0;

};


PacketsPerSecond packetRateCalc;


class NetworkingReceiver: public Task_Abstract {
public:

    NetworkingReceiver() : Task_Abstract(1000, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        commsPort.loop();

        if (commsPort.messageAvailable()) {

            packetsCounter++;

            MessageData messageInfo = commsPort.getMessageInformation();

            if (messageInfo.payloadID == eKraftMessageType_t::eKraftMessageType_String_ID) {

                KraftMessageStringPacket stringPacket;

                commsPort.getMessage(&stringPacket);

                char string[stringPacket.getStringLength()];

                stringPacket.getString(string, sizeof(string));

                Serial.println(string + String(", Rate: ") + packetRateCalc.packetRate);

            } 

            while(commsPort.messageAvailable()) commsPort.removeMessage();

        }

    }

};


class NetworkingTransmitter: public Task_Abstract {
public:

    NetworkingTransmitter() : Task_Abstract(100, eTaskPriority_t::eTaskPriority_Middle, false) {}

    void thread() {

        KraftMessageStringPacket stringPacket((String("Hello im ") + commsPort.getSelfID() + "! Time is: " + millis()).c_str());

        commsPort.sendMessage(&stringPacket, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        Serial.println("Sending: " + String("Hello im ") + commsPort.getSelfID() + "! Time is: " + millis());

        //stopTaskThreading();

    }


};

NetworkingTransmitter transmitter;
NetworkingReceiver receiver;

Observer observer;



void setup() {

    Serial.begin(115200);

}


void loop() {

    Task_Abstract::schedulerTick();
    
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