#include <Arduino.h>

#include "boards/board_v_1_0.h"
#include "KraftKontrol.h"

#include "fastlz.h"

#include "starship_specifics/starship.h"

#include "KraftKontrol/modules/navigation_modules/navigation_kalman.h"

#include "KraftKontrol/modules/sensor_modules/magnetometer_modules/hmc5883_driver.h"
#include "KraftKontrol/platforms/arduino_platform/arduino_data_manager_eeprom.h"

#include "Adafruit_SSD1306.h"


#define HEIGHT_LIMIT 50
#define DISTANCE_LIMIT 50
#define ANGLE_LIMIT (120.0f*DEGREES)


DataManager_InternalEEPROM eeprom;


SX1280Driver radio(SX1280_RFBUSY_PIN, SX1280_TXEN_PIN, SX1280_RXEN_PIN, SX1280_DIO1_PIN, SX1280_NRESET_PIN, SX1280_NSS_PIN);
KraftKommunication commsPort(radio, eKraftMessageNodeID_t::eKraftMessageNodeID_vehicle);


BME280Driver barometer(BME280_NCS_PIN, &BME280_SPIBUS);
MPU9250Driver IMU(MPU9250_INT_PIN, MPU9250_NCS_PIN, &MPU9250_SPIBUS, &eeprom);
//UbloxSerialGNSS gnss(NEO_M8Q_SERIALPORT);

I2CBusDevice_HAL magnetometerBusDevice(Wire, QMC5883Registers::QMC5883L_ADDR_DEFAULT);
QMC5883Driver magnetometer(magnetometerBusDevice, QMC5883Registers::QMC5883L_ADDR_DEFAULT, &eeprom);

//NavigationComplementaryFilter navigationmodule(&IMU, &IMU, &magnetometer, &barometer, &gnss);
NavigationKalman navigationmodule;
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
        //gyroSub.subscribe(IMU.getAccelTopic());
    }

    void thread() {

        //Vector<> vec = lpf.update(gyroSub.getItem().sensorData);

        //Serial.println(vec.toString(3));

        //Serial.println(String("IMU gyro: ") + gyroSub.getItem().sensorData.toString());

        //Serial.println(String("Acc: ") + navigationmodule.getNavigationData().data.linearAcceleration.magnitude() + ", vel: " + navigationmodule.getNavigationData().data.velocity.magnitude());

        /*if (commsPort.getNodeStatus(eKraftPacketNodeID_t::eKraftPacketNodeID_controller) == false) {
            vehicle.disarmVehicle();
        }*/

        //float angle = navigationmodule.getNavigationData().data.attitude.rotateVector(Vector(0,0,1)).getAngleTo(Vector(0,0,1));

        //Serial.println();

        //Serial.println(String("Modules: ") + Module_Abstract::getListExistingModules().getNumItems());
        
        //Serial.println(navigationmodule.getNavigationData().data.position.z);

        //Serial.println(String() + "Pos: " + navigationmodule.getNavigationData().data.position.z*5 + " Vel: " + navigationmodule.getNavigationData().data.velocity.z*3 + " Acc: " + navigationmodule.getNavigationData().data.linearAcceleration.z/5);
        //Serial.println(String() + String(navigationmodule.getNavigationData().data.position.x, 4) + "," + String(navigationmodule.getNavigationData().data.positionError.x, 4) + "," + String(navigationmodule.getNavigationData().data.position.y, 4) + "," + String(navigationmodule.getNavigationData().data.positionError.y, 4) + "," + String(navigationmodule.getNavigationData().data.position.z, 4) + "," + String(navigationmodule.getNavigationData().data.positionError.z, 4));
        //Serial.println(String((double)NOW()/SECONDS, 6));

        /*uint8_t rawData[sizeof(NavigationData)];

        memcpy(rawData, &navigationmodule.getNavigationData(), sizeof(NavigationData));

        uint32_t maxSize = sizeof(NavigationData)*1.1;
        uint8_t rawDataCompressed[maxSize];

        Serial.print("Testing compression...  ");

        //delay(100);
        int64_t startTime = NOW();
        int size = fastlz_compress_level(2, rawDataCompressed, maxSize, rawDataCompressed);

        int64_t length = NOW() - startTime;

        Serial.println(String("Raw size: ") + sizeof(NavigationData) + ", new Size: " + size + ", time: " + uint32_t(length/MICROSECONDS));*/

        //Serial.println(navigationmodule.getNavigationData().data.position.z);
        //Serial.println(IMU.gyroRate());
        //Serial.println(String("linear accel: x: ") + navigationmodule.getNavigationData().data.linearAcceleration.x + "+-" + navigationmodule.getNavigationData().data.accelerationError.x + ", y: " + navigationmodule.getNavigationData().data.linearAcceleration.y + "+-" + navigationmodule.getNavigationData().data.accelerationError.y + ", z: " + navigationmodule.getNavigationData().data.linearAcceleration.z + "+-" + navigationmodule.getNavigationData().data.accelerationError.z);
        //Serial.println(String(" Accel: x: ") + navigationmodule.getNavigationData().data.acceleration.x + "+-" + navigationmodule.getNavigationData().data.accelerationError.x + ", y: " + navigationmodule.getNavigationData().data.acceleration.y + "+-" + navigationmodule.getNavigationData().data.accelerationError.y + ", z: " + navigationmodule.getNavigationData().data.acceleration.z + "+-" + navigationmodule.getNavigationData().data.accelerationError.z);
        //Serial.println(String(" pos: x: ") + navigationmodule.getNavigationData().data.position.x + "+-" + navigationmodule.getNavigationData().data.positionError.x + ", y: " + navigationmodule.getNavigationData().data.position.y + "+-" + navigationmodule.getNavigationData().data.positionError.y + ", z: " + navigationmodule.getNavigationData().data.position.z + "+-" + String(navigationmodule.getNavigationData().data.positionError.z,6));
        //Serial.println(String(", baro Rate: ") + barometer.pressureRate());
        //Serial.print(String("att: w: ") + navigationmodule.getNavigationData().data.attitude.w + " , x: " + navigationmodule.getNavigationData().data.attitude.x + ", y: " + navigationmodule.getNavigationData().data.attitude.y + ", z: " + navigationmodule.getNavigationData().data.attitude.z + ", Calib: " + /*(magnetometer.getCalibrationStatus() == eMagCalibStatus_t::eMagCalibStatus_Calibrating ? " calibrating " : " calibrated ") + ", gnss rate: " + gnss.positionRate() + */", time: " + (double)NOW()/SECONDS + ", rate: " + IMU.gyroRate());
        //Serial.println(String(", Angular rate: x: ") + navigationmodule.getNavigationData().data.angularRate.x + "+-" + navigationmodule.getNavigationData().data.angularRateError.x + ", y: " + navigationmodule.getNavigationData().data.angularRate.y + "+-" + navigationmodule.getNavigationData().data.angularRateError.y + ", z: " + navigationmodule.getNavigationData().data.angularRate.z + "+-" + navigationmodule.getNavigationData().data.angularRateError.z);
        //Serial.println(String(", Angular Accel: x: ") + navigationmodule.getNavigationData().data.angularAcceleration.x + "+-" + String(navigationmodule.getNavigationData().data.angularAccelerationError.x,7) + ", y: " + navigationmodule.getNavigationData().data.angularAcceleration.y + "+-" + navigationmodule.getNavigationData().data.angularAccelerationError.y + ", z: " + navigationmodule.getNavigationData().data.angularAcceleration.z + "+-" + navigationmodule.getNavigationData().data.angularAccelerationError.z);
        //Serial.println(String(" Absolute: lat: ") + navigationmodule.getNavigationData().data.absolutePosition.latitude + " , long: " + navigationmodule.getNavigationData().data.absolutePosition.longitude + ", alt: " + navigationmodule.getNavigationData().data.absolutePosition.height + ", pos rate: " + gnss.positionRate());
        //Serial.println(String("GNSS status: ") + deviceStatusToString(gnss.getModuleStatus()) + String(", sats: ") + gnss.getNumSatellites() + " , rate: " + gnss.positionRate() + " , lock valid: " + (gnss.getGNSSLockValid() ? "true":"false"));
        //Serial.println(String("Hori acc: ") + gnss.getPositionAccuracy() + ", virt: " + gnss.getAltitudeAccuracy());

        //Serial.println(String("IMU gyro: ") + IMU.gyroRate() + ", accel: " + IMU.accelRate());

        //Serial.println(String("") + navigationmodule.getNavigationData().data.position.z + "  " + navigationmodule.getNavigationData().data.velocity.z + " " + constrain(navigationmodule.getNavigationData().data.acceleration.z, -4, 4));

        /*Serial.print(navigationmodule.getNavigationData().data.velocity.z);
        Serial.print(" ");
        Serial.println(navigationmodule.getNavigationData().data.position.z);*/

        //Serial.println(String("Vehicle mode: ") + vehicle.getVehicleData().vehicleMode);

        /*KraftMessageStringPacket message;

        message.setString((String("") + dynamicsModule.getForce().toString()).c_str());

        commsPort.sendMessage(&message, eKraftPacketNodeID_t::eKraftPacketNodeID_broadcast);*/
        /*
        //Checks for flight profile safety.
        if (navigationmodule.getNavigationData().data.position.z > HEIGHT_LIMIT && homeSet) {
            //vehicle.disarmVehicle();
            disarmLock = true;
        }

        Vector<> distanceVec = navigationmodule.getNavigationData().data.position;
        distanceVec.z = 0;

        if (distanceVec.magnitude() > DISTANCE_LIMIT && homeSet) {
            //vehicle.disarmVehicle();
            disarmLock = true;
        }

        float angle = Vector<>(0,0,1).getAngleTo(navigationmodule.getNavigationData().data.attitude.rotateVector(Vector<>(0,0,1)));

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
        vecMessage = navigationmodule.getNavigationData().data.angularRate;

        commsPort.getBroadcastMessageTopic().publish(vecMessage);*/

    }


    Simple_Subscriber<DataTimestamped<Vector<>>> gyroSub;

    LowPassFilter<Vector<>> lpf = 0.05;


};



class AttitudeTransmitter: public Task_Abstract {
public:

    AttitudeTransmitter() : Task_Abstract("Attitude Sender", 10, eTaskPriority_t::eTaskPriority_Middle) {}

    void thread() {

        TelemetryMessageAttitude attitudeMessage(navigationmodule.getNavigationData().data.attitude);
        commsPort.getBroadcastMessageTopic().publish(attitudeMessage);

    }


};



class PositionTransmitter: public Task_Abstract {
public:

    PositionTransmitter() : Task_Abstract("Position Sender", 20, eTaskPriority_t::eTaskPriority_Middle) {}

    void thread() {

        TelemetryMessagePosition positionMessage(navigationmodule.getNavigationData().data.position);
        commsPort.getBroadcastMessageTopic().publish(positionMessage);

    }


};



class VelocityTransmitter: public Task_Abstract {
public:

    VelocityTransmitter() : Task_Abstract("Velocity Sender", 20, eTaskPriority_t::eTaskPriority_Middle) {}

    void thread() {

        TelemetryMessageVelocity velocityMessage(navigationmodule.getNavigationData().data.velocity);
        commsPort.getBroadcastMessageTopic().publish(velocityMessage);

    }


};



class VehicleModeTransmitter: public Task_Abstract {
public:

    VehicleModeTransmitter() : Task_Abstract("Vehicle Mode Sender", 4, eTaskPriority_t::eTaskPriority_Middle) {}

    void thread() {

        TelemetryMessageVehicleModeIs vehicleMode(vehicle.getVehicleData().vehicleMode);
        commsPort.getBroadcastMessageTopic().publish(vehicleMode);

    }


};



class GNSSDataTransmitter: public Task_Abstract {
public:

    GNSSDataTransmitter() : Task_Abstract("GNSS Data Sender", 4, eTaskPriority_t::eTaskPriority_Middle) {}

    void thread() {

        //TelemetryMessageGNSSData gnssMessage(navigationmodule.getNavigationData().data.absolutePosition.latitude, navigationmodule.getNavigationData().data.absolutePosition.longitude, navigationmodule.getNavigationData().data.absolutePosition.height, gnss.getNumSatellites());
        //commsPort.getBroadcastMessageTopic().publish(gnssMessage);
        

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
                //eeprom.saveData();

            } else if (mode == eVehicleMode_t::eVehicleMode_Disarm) {

                vehicle.disarmVehicle();

            }

        }

        if (!commsPort.getNodeStatus(eKraftMessageNodeID_t::eKraftMessageNodeID_controller)) {
            vehicle.disarmVehicle();
        }

        if (vehicle.getVehicleData().vehicleMode == eVehicleMode_t::eVehicleMode_Arm && navigationmodule.getNavigationData().data.position.z > 15) vehicle.disarmVehicle();

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
GNSSDataTransmitter gnssTransmitter;

//ImAlive aliveThread;



void setup() {

    delay(2000);

    Serial.begin(115200);

    //Serial5.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    EEPROM.begin();

    eeprom.loadData();

    //eeprom.clear();
    //eeprom.saveData();

    //dynamicsModule.enableTVC(true);
    //dynamicsModule.setActuatorTesting(true);

    //CommandMessageAccelCalValues mes(Vector<>(9.5,9.41,9.03), Vector<>(-10.06,-10.1,-10.82));

    //if (!eeprom.setMessage(mes)) eeprom.newMessage(mes);

    //dynamicsModule.setControlModule(controlModule.getControlDataTopic());
    

    /*Serial.println(String("Values: ") + eeprom.getEndIndex());
    for (uint32_t i = 0; i < 50; i++) {
        Serial.print(String("-") + i + ". " + eeprom[i]);
        if ((i-1)%4 == 0) {
            uint32_t val;
            eeprom.readData(i, (uint8_t*)&val, 4);
            Serial.print(String(" = ") + val);
        }
        Serial.println();
    }*/

    //delay(2000);

    //while (1);

    navigationmodule.setGyroscopeInput(IMU);
    navigationmodule.setAccelerometerInput(IMU);
    navigationmodule.setMagnetometerInput(magnetometer);
    navigationmodule.setBarometerInput(barometer);
    //navigationmodule.setGNSSInput(gnss);

    Task_Abstract::schedulerInitTasks();


}



void loop() {

    //dynamicsModule.setActuatorTesting(true);

    Task_Abstract::schedulerTick();

    //delay(1000);

    //Serial.println((double)NOW()/SECONDS);

    /*if (millis() > 6000 && vehicle.getVehicleData().vehicleMode != eVehicleMode_t::eVehicleMode_Arm) {
        vehicle.armVehicle();
        //dynamicsModule.setActuatorTesting();
    }*/
    
}
