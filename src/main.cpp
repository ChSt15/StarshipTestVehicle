#include <Arduino.h>

#include "boards/board_v_1_0.h"
#include "KraftKontrol.h"

#include "starship_specifics/starship.h"



SX1280Driver radio(SX1280_RFBUSY_PIN, SX1280_TXEN_PIN, SX1280_RXEN_PIN, SX1280_DIO1_PIN, SX1280_NRESET_PIN, SX1280_NSS_PIN);
KraftKommunication commsPort(&radio, eKraftPacketNodeID_t::eKraftPacketNodeID_vehicle);

BME280Driver Barometer(BME280_NCS_PIN, &BME280_SPIBUS);
MPU9250Driver IMU(MPU9250_INT_PIN, MPU9250_NCS_PIN, &MPU9250_SPIBUS);
UbloxSerialGNSS gnss(&NEO_M8Q_SERIALPORT);

NavigationComplementaryFilter navigationmodule(&IMU, &IMU, &IMU/*, &Barometer, &gnss*/);
GuidanceFlyByWire guidanceModule;
StarshipControl controlModule(&guidanceModule, &navigationmodule);
StarshipDynamics dynamicsModule(&controlModule, &navigationmodule);

KraftKonnectNetwork network(&commsPort);

Starship vehicle(&guidanceModule, &navigationmodule, &controlModule, &dynamicsModule);


uint32_t packetsCounter = 0;
uint32_t packetsSent = 0;



void radioControlPacketEventHandler() {


    KraftMessageRCChannels message;

    if (!commsPort.getMessage(&message)) return;

    int16_t channels[15];

    message.getChannelAll(channels);

    float x = (float)(channels[0]-1500)/500.0*15*DEGREES;
    float y = (float)(channels[1]-1500)/500.0*15*DEGREES;

    return;

    if (x != 0 && y != 0) {

        Vector<> rotVec = Vector<>(x, y, 0);

        Quaternion<> attitude = Quaternion<>(rotVec.copy().normalize(), rotVec.magnitude());

        guidanceModule.setAttitude(attitude);
        guidanceModule.setAngularRate(Vector<>(0,0,0));
        guidanceModule.setPosition(navigationmodule.getNavigationData().position);
        guidanceModule.setVelocity(Vector<>(0,0,0));

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

        //float angle = navigationmodule.getNavigationData().attitude.rotateVector(Vector(0,0,1)).getAngleTo(Vector(0,0,1));

        Serial.println();

        //Serial.println(navigationmodule.getNavigationData().position.z);
        //Serial.println(IMU.gyroRate());
        //Serial.println(String("linear accel: x: ") + navigationmodule.getNavigationData().linearAcceleration.x + ", y: " + navigationmodule.getNavigationData().linearAcceleration.x + ", z: " + navigationmodule.getNavigationData().linearAcceleration.z);
        //Serial.println(String("speed: x: ") + navigationmodule.getNavigationData().velocity.x + ", y: " + navigationmodule.getNavigationData().velocity.x + ", z: " + navigationmodule.getNavigationData().velocity.z);
        //Serial.println(String("position: x: ") + navigationmodule.getNavigationData().position.x + ", y: " + navigationmodule.getNavigationData().position.x + ", z: " + navigationmodule.getNavigationData().position.z);
        Serial.println(String("Attitude: w: ") + navigationmodule.getNavigationData().attitude.w + " , x: " + navigationmodule.getNavigationData().attitude.x + ", y: " + navigationmodule.getNavigationData().attitude.x + ", z: " + navigationmodule.getNavigationData().attitude.z);
        //Serial.println(String("GNSS status: ") + deviceStatusToString(gnss.getModuleStatus()) + String(", sats: ") + gnss.getNumSatellites() + " , rate: " + gnss.positionRate() + " , lock valid: " + (gnss.getGNSSLockValid() ? "true":"false"));

    }


};



class NetworkingTransmitter: public Task_Abstract {
public:

    NetworkingTransmitter() : Task_Abstract(10, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {


        if (commsPort.networkBusy()) return;

        KraftMessageAttitude attitudeMessage(navigationmodule.getNavigationData().attitude);
        //commsPort.sendMessage(&attitudeMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        KraftMessagePosition positionMessage(navigationmodule.getNavigationData().position, navigationmodule.getNavigationData().timestamp);
        commsPort.sendMessage(&positionMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        KraftMessageVehicleModeIs vehicleMode(vehicle.getVehicleData().vehicleMode);
        //commsPort.sendMessage(&vehicleMode, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);


        if (true) {

            WorldPosition worldPosition; uint32_t time;
            //gnss.getPosition(&worldPosition, &time);

            //KraftMessageGNSSData gnssMessage(worldPosition, gnss.getNumSatellites());

            //commsPort.sendMessage(&gnssMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        }



        //KraftMessageStringPacket stringPacket((String("Hello im ") + commsPort.getSelfID() + "! Time is: " + millis()).c_str());

        //commsPort.sendMessage(&stringPacket, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        //Serial.println("Sending: " + String("Hello im ") + commsPort.getSelfID() + "! Time is: " + millis());

        //KraftMessageFullKinematics message(navigationmodule.getNavigationData());

        //commsPort.sendMessage(&message, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        /*if (gnss.positionAvailable()) {

            WorldPosition worldPosition; uint32_t time;
            gnss.getPosition(&worldPosition, &time);

            KraftMessageGNSSData gnssMessage(worldPosition, gnss.getNumSatellites());

            commsPort.sendMessage(&gnssMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);

        }*/

        

        

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
