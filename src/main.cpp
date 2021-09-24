#include <Arduino.h>

#include "boards/board_v_1_0.h"
#include "KraftKontrol.h"

#include "starship_specifics/starship.h"

#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/hmc5883_driver.h"


#define HEIGHT_LIMIT 50
#define DISTANCE_LIMIT 50
#define ANGLE_LIMIT (120.0f*DEGREES)


SX1280Driver radio(SX1280_RFBUSY_PIN, SX1280_TXEN_PIN, SX1280_RXEN_PIN, SX1280_DIO1_PIN, SX1280_NRESET_PIN, SX1280_NSS_PIN);
KraftKommunication commsPort(radio, eKraftMessageNodeID_t::eKraftMessageNodeID_vehicle);


BME280Driver barometer(BME280_NCS_PIN, &BME280_SPIBUS);
MPU9250Driver IMU(MPU9250_INT_PIN, MPU9250_NCS_PIN, &MPU9250_SPIBUS);
UbloxSerialGNSS gnss(NEO_M8Q_SERIALPORT);

I2CBusDevice_HAL magnetometerBusDevice(Wire, QMC5883Registers::QMC5883L_ADDR_DEFAULT);
QMC5883Driver magnetometer = QMC5883Driver(magnetometerBusDevice, QMC5883Registers::QMC5883L_ADDR_DEFAULT);

NavigationComplementaryFilter navigationmodule(&IMU, &IMU, &magnetometer, &barometer, &gnss);
GuidanceFlyByWire guidanceModule;
StarshipControl controlModule(guidanceModule, navigationmodule);
StarshipDynamics dynamicsModule(controlModule, navigationmodule);

Starship vehicle(&guidanceModule, &navigationmodule, &controlModule, &dynamicsModule);



/*extern unsigned long _heap_end;
extern char *__brkval;
uint32_t FreeMem(){ // for Teensy 4.1
  char* p = (char*) malloc(10000); // size should be quite big, to avoid allocating fragment!
  free(p);
  return (char *)&_heap_end - p; // __brkval;
}*/



class Observer: public Task_Abstract {
public:

    Observer() : Task_Abstract("Observer", 100, eTaskPriority_t::eTaskPriority_Middle) {}

    void init() {

    }

    void thread() {

        //Serial.println(String("IMU gyro: ") + gyroSub.getItem().sensorData.toString());

        /*if (commsPort.getNodeStatus(eKraftPacketNodeID_t::eKraftPacketNodeID_controller) == false) {
            vehicle.disarmVehicle();
        }*/

        //float angle = navigationmodule.getNavigationData().attitude.rotateVector(Vector(0,0,1)).getAngleTo(Vector(0,0,1));

        //Serial.println();

        //Serial.println(String("Modules: ") + Module_Abstract::getListExistingModules().getNumItems());
        
        //Serial.println(navigationmodule.getNavigationData().position.z);

        //Serial.println(String() + "Pos: " + navigationmodule.getNavigationData().position.z*5 + " Vel: " + navigationmodule.getNavigationData().velocity.z*3 + " Acc: " + navigationmodule.getNavigationData().linearAcceleration.z/5);
        //Serial.println(String() + String(navigationmodule.getNavigationData().position.x, 4) + "," + String(navigationmodule.getNavigationData().positionError.x, 4) + "," + String(navigationmodule.getNavigationData().position.y, 4) + "," + String(navigationmodule.getNavigationData().positionError.y, 4) + "," + String(navigationmodule.getNavigationData().position.z, 4) + "," + String(navigationmodule.getNavigationData().positionError.z, 4));


        //Serial.println(navigationmodule.getNavigationData().position.z);
        //Serial.println(IMU.gyroRate());
        //Serial.println(String("linear accel: x: ") + navigationmodule.getNavigationData().linearAcceleration.x + ",\ty: " + navigationmodule.getNavigationData().linearAcceleration.x + ", \tz: " + navigationmodule.getNavigationData().linearAcceleration.z);
        //Serial.println(String(" Accel: x: ") + navigationmodule.getNavigationData().acceleration.x + "+-" + navigationmodule.getNavigationData().accelerationError.x + ", y: " + navigationmodule.getNavigationData().acceleration.y + "+-" + navigationmodule.getNavigationData().accelerationError.y + ", z: " + navigationmodule.getNavigationData().acceleration.z + "+-" + navigationmodule.getNavigationData().accelerationError.z);
        //Serial.println(String(" pos: x: ") + navigationmodule.getNavigationData().position.x + "+-" + navigationmodule.getNavigationData().positionError.x + ", y: " + navigationmodule.getNavigationData().position.y + "+-" + navigationmodule.getNavigationData().positionError.y + ", z: " + navigationmodule.getNavigationData().position.z + "+-" + String(navigationmodule.getNavigationData().positionError.z,6));
        //Serial.println(String(", baro Rate: ") + barometer.pressureRate());
        //Serial.println(String("att: w: ") + navigationmodule.getNavigationData().attitude.w + " , x: " + navigationmodule.getNavigationData().attitude.x + ", y: " + navigationmodule.getNavigationData().attitude.y + ", z: " + navigationmodule.getNavigationData().attitude.z + ", Calib: " + /*(magnetometer.getCalibrationStatus() == eMagCalibStatus_t::eMagCalibStatus_Calibrating ? " calibrating " : " calibrated ") + ", gnss rate: " + gnss.positionRate() + */", time: " + (double)NOW()/SECONDS + ", rate: " + IMU.gyroRate());
        //Serial.println(String(", Angular rate: x: ") + navigationmodule.getNavigationData().angularRate.x + "+-" + navigationmodule.getNavigationData().angularRateError.x + ", y: " + navigationmodule.getNavigationData().angularRate.y + "+-" + navigationmodule.getNavigationData().angularRateError.y + ", z: " + navigationmodule.getNavigationData().angularRate.z + "+-" + navigationmodule.getNavigationData().angularRateError.z);
        //Serial.println(String(" Absolute: lat: ") + navigationmodule.getNavigationData().absolutePosition.latitude + " , long: " + navigationmodule.getNavigationData().absolutePosition.longitude + ", alt: " + navigationmodule.getNavigationData().absolutePosition.height + ", pos rate: " + gnss.positionRate());
        //Serial.println(String("GNSS status: ") + deviceStatusToString(gnss.getModuleStatus()) + String(", sats: ") + gnss.getNumSatellites() + " , rate: " + gnss.positionRate() + " , lock valid: " + (gnss.getGNSSLockValid() ? "true":"false"));
        //Serial.println(String("Hori acc: ") + gnss.getPositionAccuracy() + ", virt: " + gnss.getAltitudeAccuracy());

        //Serial.println(String("IMU gyro: ") + IMU.gyroRate() + ", accel: " + IMU.accelRate());

        //Serial.println(String("") + navigationmodule.getNavigationData().position.z + "  " + navigationmodule.getNavigationData().velocity.z + " " + constrain(navigationmodule.getNavigationData().acceleration.z, -4, 4));

        /*Serial.print(navigationmodule.getNavigationData().velocity.z);
        Serial.print(" ");
        Serial.println(navigationmodule.getNavigationData().position.z);*/

        //Serial.println(String("Vehicle mode: ") + vehicle.getVehicleData().vehicleMode);

        /*KraftMessageStringPacket message;

        message.setString((String("") + dynamicsModule.getForce().toString()).c_str());

        commsPort.sendMessage(&message, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);*/
        /*
        //Checks for flight profile safety.
        if (navigationmodule.getNavigationData().position.z > HEIGHT_LIMIT && homeSet) {
            //vehicle.disarmVehicle();
            disarmLock = true;
        }

        Vector<> distanceVec = navigationmodule.getNavigationData().position;
        distanceVec.z = 0;

        if (distanceVec.magnitude() > DISTANCE_LIMIT && homeSet) {
            //vehicle.disarmVehicle();
            disarmLock = true;
        }

        float angle = Vector<>(0,0,1).getAngleTo(navigationmodule.getNavigationData().attitude.rotateVector(Vector<>(0,0,1)));

        if (angle > ANGLE_LIMIT) {
            //vehicle.disarmVehicle();
            disarmLock = true;
        }

*/

        //volatile int8_t test[100];

        //delete[] test;

        //Serial.println(String("Free RAM: ") + FreeMem() + ", time: " + (float)NOW()/SECONDS + ", test 1: " + test[0]);

        //delete[] test;

        /*TelemetryMessageVelocity vecMessage;
        vecMessage = navigationmodule.getNavigationData().angularRate;

        commsPort.getBroadcastMessageTopic().publish(vecMessage);*/

    }


    Simple_Subscriber<SensorTimestamp<Vector<>>> gyroSub;


};



class AttitudeTransmitter: public Task_Abstract {
public:

    AttitudeTransmitter() : Task_Abstract("Attitude Sender", 5, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        TelemetryMessageAttitude attitudeMessage(navigationmodule.getNavigationData().attitude);
        commsPort.getBroadcastMessageTopic().publish(attitudeMessage);

    }


};



class PositionTransmitter: public Task_Abstract {
public:

    PositionTransmitter() : Task_Abstract("Position Sender", 20, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        TelemetryMessagePosition positionMessage(navigationmodule.getNavigationData().position);
        commsPort.getBroadcastMessageTopic().publish(positionMessage);

    }


};



class VelocityTransmitter: public Task_Abstract {
public:

    VelocityTransmitter() : Task_Abstract("Velocity Sender", 20, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        TelemetryMessageVelocity velocityMessage(navigationmodule.getNavigationData().velocity);
        commsPort.getBroadcastMessageTopic().publish(velocityMessage);

    }


};



class VehicleModeTransmitter: public Task_Abstract {
public:

    VehicleModeTransmitter() : Task_Abstract("Vehicle Mode Sender", 5, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        TelemetryMessageVehicleModeIs vehicleMode(vehicle.getVehicleData().vehicleMode);
        commsPort.getBroadcastMessageTopic().publish(vehicleMode);

    }


};



class GNSSDataTransmitter: public Task_Abstract {
public:

    GNSSDataTransmitter() : Task_Abstract("GNSS Data Sender", 4, eTaskPriority_t::eTaskPriority_Middle, true) {}

    void thread() {

        /*
        if (commsPort.networkBusy()) return;

        KraftMessageGNSSData gnssMessage(navigationmodule.getNavigationData().absolutePosition, gnss.getNumSatellites());
        commsPort.sendMessage(&gnssMessage, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);
        */

    }


};



class VehicleModeSetter: public Task_Abstract {
public:

    VehicleModeSetter() : Task_Abstract("Vehicle Mode Setter", 10, eTaskPriority_t::eTaskPriority_Middle) {}

    CommandMessageVehicleModeSet vehicleModeSet;
    KraftMessage_Subscriber subr;

    void init() {

        subr.addReceiverMessage(vehicleModeSet);
        subr.subscribe(commsPort.getReceivedMessageTopic());
        subr.setTaskToResume(*this);

    }

    void thread() {

        if (subr.isDataNew()) {

            eVehicleMode_t mode = vehicleModeSet.getData(); 

            char modeName[20];
            getVehicleModeString(modeName, 20, mode);

            //Serial.println(String("Got Mode: ") + modeName);

            if (mode == eVehicleMode_t::eVehicleMode_Arm) {

                vehicle.armVehicle();

            } else if (mode == eVehicleMode_t::eVehicleMode_Disarm) {

                vehicle.disarmVehicle();

            }

        }

        if (!commsPort.getNodeStatus(eKraftMessageNodeID_t::eKraftMessageNodeID_controller)) {
            vehicle.disarmVehicle();
        }

        if (vehicle.getVehicleData().vehicleMode == eVehicleMode_t::eVehicleMode_Arm && navigationmodule.getNavigationData().position.z > 15) vehicle.disarmVehicle();

    } 

};



class VehiclePositionSetter: public Task_Abstract {
public:

    VehiclePositionSetter() : Task_Abstract("Position Setter", 1, eTaskPriority_t::eTaskPriority_Middle) {}

    CommandMessagePositionSet vehiclePosSet;
    KraftMessage_Subscriber subr;

    void init() {

        subr.addReceiverMessage(vehiclePosSet);
        subr.subscribe(commsPort.getReceivedMessageTopic());
        subr.setTaskToResume(*this);

    }

    void thread() {

        Vector<> pos = vehiclePosSet.getVector();
        guidanceModule.setPosition(pos);

        //Serial.println(String("Got pos: ") + pos.toString());

    } 

};



class ImAlive: public Task_Abstract {
public:

    ImAlive() : Task_Abstract("Im Alive", 1, eTaskPriority_t::eTaskPriority_Realtime) {}

    void thread() {

        Serial.println(String("Num tasks: ") + Task_Abstract::getTaskList().getNumItems());
        Serial.println("Task usages: ");
        Serial.println(String("CPU Usage: ") + String(Task_Abstract::getSchedulerSystemUsage()*100.0, 2));
        for (uint32_t i = 0; i < Task_Abstract::getTaskList().getNumItems(); i++) {
            Serial.println(String() + "    - Usage: " + String(Task_Abstract::getTaskList()[i]->getTaskSystemUsage()*100, 2) + ", rate: " + Task_Abstract::getTaskList()[i]->getLoopRate() + "\t, Name: " + Task_Abstract::getTaskList()[i]->getTaskName());
        }

    }

    void init() {

        Serial.println("IM ALIVE. IVE BEEN INITIALISED!");

    }

    void removal() {

        Serial.println("WHY HAVE YOU FORSAKEN MEEEE!");

    }


};


Observer observer;
AttitudeTransmitter attTransmitter;
PositionTransmitter posTransmitter;
VelocityTransmitter velTransmitter;
VehicleModeTransmitter modeTransmitter;

VehiclePositionSetter posSetter;
VehicleModeSetter modeSetter;
//GNSSDataTransmitter gnssTransmitter;

//ImAlive aliveThread;



void setup() {

    Serial.begin(115200);

    //Serial5.begin(115200);

    //Wire.begin();

    delay(2000);

    Task_Abstract::schedulerInitTasks();


}



void loop() {

    Task_Abstract::schedulerTick();

    //delay(1000);

    //Serial.println((double)NOW()/SECONDS);

    /*if (millis() > 6000 && vehicle.getVehicleData().vehicleMode != eVehicleMode_t::eVehicleMode_Arm) {
        vehicle.armVehicle();
        //dynamicsModule.setActuatorTesting();
    }*/
    
}
