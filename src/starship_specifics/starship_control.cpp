#include "starship_control.h"



void StarshipControl::thread() {

    float dTime = float(NOW() - lastLoopTimestamp_)/SECONDS;
    lastLoopTimestamp_ = NOW();


    //Position control section
    //controlOutput_.data.force.x = 0;
    //controlOutput_.data.force.y = 0;
    //controlOutput_.data.force.z = (1.0-navigationData.data.position.z)*positionPF_.z - navigationData.data.velocity.z*velocityPF_.z + 9.81*vehicleMass_; 
    //controlOutput_.data.force = navigationData.data.attitude.copy().conjugate().rotateVector(controlOutput_.data.force); //Rotate to local coordinate system

    DataTimestamped<NavigationData> navigationData = navigationSub_.getItem();
    ControlData controlSetpoint = guidanceSub_.getItem();

    {  
        
        //calculate error
        Vector<> positionControllerOutput = 0;
        Vector<> positionError = controlSetpoint.position - navigationData.data.position;


        //Calculate Velocity I term
        positionIValue_ += positionIValue_*positionIF_*dTime;

        //Calculate output for limiting
        positionControllerOutput = positionIValue_ + positionError*positionPF_ - navigationData.data.velocity*positionDF_;

        //Anti windup for output
        if (positionControllerOutput.x > positionLimit_.x) {
            positionIValue_.x -= positionControllerOutput.x - positionLimit_.x; //Remove saturation from I according to overthreshold
            positionIValue_.x = max(positionIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
        } else if (positionControllerOutput.x < -positionLimit_.x) {
            positionIValue_.x -= positionControllerOutput.x + positionLimit_.x; //Remove saturation from I according to overthreshold
            positionIValue_.x = min(positionIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
        }
        if (positionControllerOutput.y > positionLimit_.y) {
            positionIValue_.y -= positionControllerOutput.y - positionLimit_.y; //Remove saturation from I according to overthreshold
            positionIValue_.y = max(positionIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
        } else if (positionControllerOutput.y < -positionLimit_.y) {
            positionIValue_.y -= positionControllerOutput.y + positionLimit_.y; //Remove saturation from I according to overthreshold
            positionIValue_.y = min(positionIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
        }
        if (positionControllerOutput.z > positionLimit_.z) {
            positionIValue_.z -= positionControllerOutput.z - positionLimit_.z; //Remove saturation from I according to overthreshold
            positionIValue_.z = max(positionIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
        } else if (positionControllerOutput.z < -positionLimit_.z) {
            positionIValue_.z -= positionControllerOutput.z + positionLimit_.z; //Remove saturation from I according to overthreshold
            positionIValue_.z = min(positionIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
        }

        //Calculate new output
        positionControllerOutput = positionIValue_ + positionError*positionPF_ - navigationData.data.velocity*positionDF_;


        //Velocity controller
        Vector<> velocityControllerOutput = 0;
    	Vector<> velocityError = positionControllerOutput + controlSetpoint.velocity - navigationData.data.velocity;

        //Calculate Velocity I term
        velocityIValue_ += velocityError*velocityIF_*dTime;

        //Calculate output for limiting
        velocityControllerOutput = velocityIValue_ + velocityError*velocityPF_ - navigationData.data.linearAcceleration*velocityDF_;

        //Anti windup for output
        if (velocityControllerOutput.x > velocityLimit_.x) {
            velocityIValue_.x -= velocityControllerOutput.x - velocityLimit_.x; //Remove saturation from I according to overthreshold
            velocityIValue_.x = max(velocityIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
        } else if (velocityControllerOutput.x < -velocityLimit_.x) {
            velocityIValue_.x -= velocityControllerOutput.x + velocityLimit_.x; //Remove saturation from I according to overthreshold
            velocityIValue_.x = min(velocityIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
        }
        if (velocityControllerOutput.y > velocityLimit_.y) {
            velocityIValue_.y -= velocityControllerOutput.y - velocityLimit_.y; //Remove saturation from I according to overthreshold
            velocityIValue_.y = max(velocityIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
        } else if (velocityControllerOutput.y < -velocityLimit_.y) {
            velocityIValue_.y -= velocityControllerOutput.y + velocityLimit_.y; //Remove saturation from I according to overthreshold
            velocityIValue_.y = min(velocityIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
        }
        if (velocityControllerOutput.z > velocityLimit_.z) {
            velocityIValue_.z -= velocityControllerOutput.z - velocityLimit_.z; //Remove saturation from I according to overthreshold
            velocityIValue_.z = max(velocityIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
        } else if (velocityControllerOutput.z < -velocityLimit_.z) {
            velocityIValue_.z -= velocityControllerOutput.z + velocityLimit_.z; //Remove saturation from I according to overthreshold
            velocityIValue_.z = min(velocityIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
        }

        //Calculate new output
        velocityControllerOutput = velocityIValue_ + velocityError*velocityPF_ - navigationData.data.linearAcceleration*velocityDF_;

        
        //Include external forces e.g. gravity
        Vector<> forceBuf = velocityControllerOutput + Vector<>(0,0,9.81)*1.0f;

        //Calulate rotation
        Vector<> rotAxis = Vector<>(0,0,1).cross(forceBuf).normalize();
        float rotAngle = Vector<>(0,0,1).getAngleTo(forceBuf);
        rotAngle = constrain(rotAngle, -15.0f*DEGREES, 15.0f*DEGREES); //Constrain rotation
        FML::Quaternion<> rotation = FML::Quaternion<>(rotAxis, rotAngle);

        //calculate force output
        float cosLimit = max(cos(rotAngle), 0);
        Vector<> force = rotation.rotateVector(Vector<>(0,0,1))*forceBuf.magnitude()*cosLimit;

        //Transform into local coordinate system
        force = navigationData.data.attitude.rotateVector(force);

        //Update output
        controlOutput_.data.force = force;

        //Update attitude setpoint
        controlSetpoint.attitude = rotation;

        //controlSetpoint.attitude = FML::Quaternion<>(Vector<>(0,1,0), 90*DEGREES);

        //Serial.println(controlSetpoint.attitude.toString());

        
        //float angle; Vector<> axis;

        //rotation.getAxisAngle(axis, angle);
        //Serial.println(String("Angle: ") + angle/DEGREES + ", axis: " + axis.toString());


    }

    //Position control section
    /*if (controlSetpoint.positionControlMode == eControlMode_t::eControlMode_Disable) {

        controlOutput_.data.force = 0;

    } else {    

        controlOutput_.data.force = 0;

        ControlData setpoint = *controlSetpoint;
        KinematicData ispoint = *navigationData;

        Vector<> positionOutput(0);
        Vector<> velOutput(0);
        Vector<> accelOutput(0);

        Vector<> outputTotal(0);

        if (setpoint.positionControlMode == eControlMode_t::eControlMode_Position || setpoint.positionControlMode == eControlMode_t::eControlMode_Velocity_Position || setpoint.positionControlMode == eControlMode_t::eControlMode_Acceleration_Velocity_Position) {

            Vector<> error = (setpoint.position - ispoint.position);

            positionIValue_ += error.compWiseMulti(positionIF_);

            Vector<> positionOutput = error.compWiseMulti(positionPF_) + ispoint.attitude.rotateVector(setpoint.velocity - ispoint.velocity)*positionDF_ + positionIValue_;

            if (positionOutput.x > positionLimit_.x) {
                positionIValue_.x -= positionOutput.x - positionLimit_.x; //Remove saturation from I according to overthreshold
                positionIValue_.x = max(positionIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (positionOutput.x < -positionLimit_.x) {
                positionIValue_.x -= positionOutput.x + positionLimit_.x; //Remove saturation from I according to overthreshold
                positionIValue_.x = min(positionIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (positionOutput.y > positionLimit_.y) {
                positionIValue_.y -= positionOutput.y - positionLimit_.y; //Remove saturation from I according to overthreshold
                positionIValue_.y = max(positionIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (positionOutput.y < -positionLimit_.y) {
                positionIValue_.y -= positionOutput.y + positionLimit_.y; //Remove saturation from I according to overthreshold
                positionIValue_.y = min(positionIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (positionOutput.z > positionLimit_.z) {
                positionIValue_.z -= positionOutput.z - positionLimit_.z; //Remove saturation from I according to overthreshold
                positionIValue_.z = max(positionIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (positionOutput.z < -positionLimit_.z) {
                positionIValue_.z -= positionOutput.z + positionLimit_.z; //Remove saturation from I according to overthreshold
                positionIValue_.z = min(positionIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            attitudeOutput = error.compWiseMulti(attitudePF_) + (setpoint.angularRate - navigationData.data.angularRate).compWiseMulti(attitudeDF_) + attitudeIValue_; //Recalculate new output

            //Constrain output
            attitudeOutput.x = constrain(attitudeOutput.x, -attitudeLimit_.x, attitudeLimit_.x);
            attitudeOutput.y = constrain(attitudeOutput.y, -attitudeLimit_.y, attitudeLimit_.y);
            attitudeOutput.z = constrain(attitudeOutput.z, -attitudeLimit_.z, attitudeLimit_.z);

            //attitudeOutput = navigationData.data.attitude.rotateVector(attitudeOutput);

            //Serial.println(String("Output: x: ") + attitudeOutput.x + ", y: " + attitudeOutput.y + ", z: " + attitudeOutput.z);

            if (attitudePassThrough_) outputTotal += attitudeOutput;
            else setpoint.angularRate += attitudeOutput;

        }

        if (setpoint.positionControlMode == eControlMode_t::eControlMode_Velocity || setpoint.positionControlMode == eControlMode_t::eControlMode_Velocity_Position || setpoint.positionControlMode == eControlMode_t::eControlMode_Acceleration_Velocity_Position) {

            Vector<> error = navigationData.data.attitude.copy().conjugate().rotateVector(setpoint.angularRate - navigationData.data.angularRate); //Calculate setpoint error and then rotate to local coordinate system.

            angVelIValue_ += error.compWiseMulti(angVelIF_);

            Vector<> angVelOutput = error.compWiseMulti(angVelPF_) + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) + angVelIValue_;

            if (angVelOutput.x > angVelLimit_.x) {
                angVelIValue_.x -= angVelOutput.x - angVelLimit_.x; //Remove saturation from I according to overthreshold
                angVelIValue_.x = max(angVelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.x < -angVelLimit_.x) {
                angVelIValue_.x -= angVelOutput.x + angVelLimit_.x; //Remove saturation from I according to overthreshold
                angVelIValue_.x = min(angVelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angVelOutput.y > angVelLimit_.y) {
                angVelIValue_.y -= angVelOutput.y - angVelLimit_.y; //Remove saturation from I according to overthreshold
                angVelIValue_.y = max(angVelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.y < -angVelLimit_.y) {
                angVelIValue_.y -= angVelOutput.y + angVelLimit_.y; //Remove saturation from I according to overthreshold
                angVelIValue_.y = min(angVelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angVelOutput.z > angVelLimit_.z) {
                angVelIValue_.z -= angVelOutput.z - angVelLimit_.z; //Remove saturation from I according to overthreshold
                angVelIValue_.z = max(angVelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.z < -angVelLimit_.z) {
                angVelIValue_.z -= angVelOutput.z + angVelLimit_.z; //Remove saturation from I according to overthreshold
                angVelIValue_.z = min(angVelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            angVelOutput = error.compWiseMulti(angVelPF_) + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) + angVelIValue_; //Recalculate new output

            //Constrain output
            angVelOutput.x = constrain(angVelOutput.x, -angVelLimit_.x, angVelLimit_.x);
            angVelOutput.y = constrain(angVelOutput.y, -angVelLimit_.y, angVelLimit_.y);
            angVelOutput.z = constrain(angVelOutput.z, -angVelLimit_.z, angVelLimit_.z);

            if (angVelPassThrough_) outputTotal += angVelOutput;
            else setpoint.angularAcceleration += angVelOutput;
            

        }

        if (setpoint.positionControlMode == eControlMode_t::eControlMode_Acceleration || setpoint.positionControlMode == eControlMode_t::eControlMode_Acceleration_Velocity || setpoint.positionControlMode == eControlMode_t::eControlMode_Acceleration_Velocity_Position) {

            Vector<> error = navigationData.data.attitude.copy().conjugate().rotateVector(setpoint.angularAcceleration - navigationData.data.angularAcceleration); //Calculate setpoint error and then rotate to local coordinate system.

            angAccelIValue_ += error.compWiseMulti(angAccelIF_);

            Vector<> angAccelOutput = error.compWiseMulti(angAccelPF_) + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) currently not implemented  + angAccelIValue_;

            if (angAccelOutput.x > angAccelLimit_.x) {
                angAccelIValue_.x -= angAccelOutput.x - angAccelLimit_.x; //Remove saturation from I according to overthreshold
                angAccelIValue_.x = max(angAccelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.x < -angAccelLimit_.x) {
                angAccelIValue_.x -= angAccelOutput.x + angAccelLimit_.x; //Remove saturation from I according to overthreshold
                angAccelIValue_.x = min(angAccelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angAccelOutput.y > angAccelLimit_.y) {
                angAccelIValue_.y -= angAccelOutput.y - angAccelLimit_.y; //Remove saturation from I according to overthreshold
                angAccelIValue_.y = max(angAccelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.y < -angAccelLimit_.y) {
                angAccelIValue_.y -= angAccelOutput.y + angAccelLimit_.y; //Remove saturation from I according to overthreshold
                angAccelIValue_.y = min(angAccelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angAccelOutput.z > angAccelLimit_.z) {
                angAccelIValue_.z -= angAccelOutput.z - angAccelLimit_.z; //Remove saturation from I according to overthreshold
                angAccelIValue_.z = max(angAccelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.z < -angAccelLimit_.z) {
                angAccelIValue_.z -= angAccelOutput.z + angAccelLimit_.z; //Remove saturation from I according to overthreshold
                angAccelIValue_.z = min(angAccelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            angAccelOutput = error.compWiseMulti(angAccelPF_) + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) currently not implemented  + angAccelIValue_; //Recalculate new output

            //Constrain output
            angAccelOutput.x = constrain(angAccelOutput.x, -angAccelLimit_.x, angAccelLimit_.x);
            angAccelOutput.y = constrain(angAccelOutput.y, -angAccelLimit_.y, angAccelLimit_.y);
            angAccelOutput.z = constrain(angAccelOutput.z, -angAccelLimit_.z, angAccelLimit_.z);

        }

        //controlOutput_.data.torqe = outputTotal;
        //Serial.println(String("Test: x: ") + controlOutput_.data.torqe.x + ", y: " + controlOutput_.data.torqe.y + ", z: " + controlOutput_.data.torqe.z);

    }*/


    
    //Attitude control section
    if (!(controlSetpoint.attitudeControlMode.accelerationControl || controlSetpoint.attitudeControlMode.velocityControl || controlSetpoint.attitudeControlMode.positionControl)) {

        controlOutput_.data.torqe = 0;

    } else {    

        controlOutput_.data.torqe = 0;

        ControlData setpoint = controlSetpoint;

        Vector<> attitudeOutput(0);
        Vector<> angVelOutput(0);
        Vector<> angAccelOutput(0);

        Vector<> outputTotal(0);

        if (setpoint.attitudeControlMode.positionControl) {

            Vector<> error = navigationData.data.attitude.copy().conjugate().rotateVector((setpoint.attitude.normalize(true)^navigationData.data.attitude.copy().conjugate().normalize(true)).normalize(true).toVector()); //Error is calculated here already in local coordinate system.

            //Serial.println(String("Error: x: ") + error.x + ", y: " + error.y + ", z: " + error.z);

            attitudeIValue_ += error.compWiseMulti(attitudeIF_);

            Vector<> attitudeOutput = error.compWiseMulti(attitudePF_) + navigationData.data.attitude.rotateVector(setpoint.angularRate - navigationData.data.angularRate).compWiseMulti(attitudeDF_) + attitudeIValue_;

            if (attitudeOutput.x > attitudeLimit_.x) {
                attitudeIValue_.x -= attitudeOutput.x - attitudeLimit_.x; //Remove saturation from I according to overthreshold
                attitudeIValue_.x = max(attitudeIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (attitudeOutput.x < -attitudeLimit_.x) {
                attitudeIValue_.x -= attitudeOutput.x + attitudeLimit_.x; //Remove saturation from I according to overthreshold
                attitudeIValue_.x = min(attitudeIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (attitudeOutput.y > attitudeLimit_.y) {
                attitudeIValue_.y -= attitudeOutput.y - attitudeLimit_.y; //Remove saturation from I according to overthreshold
                attitudeIValue_.y = max(attitudeIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (attitudeOutput.y < -attitudeLimit_.y) {
                attitudeIValue_.y -= attitudeOutput.y + attitudeLimit_.y; //Remove saturation from I according to overthreshold
                attitudeIValue_.y = min(attitudeIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (attitudeOutput.z > attitudeLimit_.z) {
                attitudeIValue_.z -= attitudeOutput.z - attitudeLimit_.z; //Remove saturation from I according to overthreshold
                attitudeIValue_.z = max(attitudeIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (attitudeOutput.z < -attitudeLimit_.z) {
                attitudeIValue_.z -= attitudeOutput.z + attitudeLimit_.z; //Remove saturation from I according to overthreshold
                attitudeIValue_.z = min(attitudeIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            attitudeOutput = error.compWiseMulti(attitudePF_) + navigationData.data.attitude.rotateVector(setpoint.angularRate - navigationData.data.angularRate).compWiseMulti(attitudeDF_) + attitudeIValue_; //Recalculate new output

            //Constrain output
            attitudeOutput.x = constrain(attitudeOutput.x, -attitudeLimit_.x, attitudeLimit_.x);
            attitudeOutput.y = constrain(attitudeOutput.y, -attitudeLimit_.y, attitudeLimit_.y);
            attitudeOutput.z = constrain(attitudeOutput.z, -attitudeLimit_.z, attitudeLimit_.z);

            //attitudeOutput = navigationData.data.attitude.rotateVector(attitudeOutput);

            //Serial.println(String("Output: x: ") + attitudeOutput.x + ", y: " + attitudeOutput.y + ", z: " + attitudeOutput.z);

            if (attitudePassThrough_) outputTotal += attitudeOutput;
            else setpoint.angularRate += attitudeOutput;

        }

        if (setpoint.attitudeControlMode.velocityControl) {

            Vector<> error = navigationData.data.attitude.copy().conjugate().rotateVector(setpoint.angularRate - navigationData.data.angularRate); //Calculate setpoint error and then rotate to local coordinate system.

            angVelIValue_ += error.compWiseMulti(angVelIF_);

            Vector<> angVelOutput = error.compWiseMulti(angVelPF_) + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) + angVelIValue_;

            if (angVelOutput.x > angVelLimit_.x) {
                angVelIValue_.x -= angVelOutput.x - angVelLimit_.x; //Remove saturation from I according to overthreshold
                angVelIValue_.x = max(angVelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.x < -angVelLimit_.x) {
                angVelIValue_.x -= angVelOutput.x + angVelLimit_.x; //Remove saturation from I according to overthreshold
                angVelIValue_.x = min(angVelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angVelOutput.y > angVelLimit_.y) {
                angVelIValue_.y -= angVelOutput.y - angVelLimit_.y; //Remove saturation from I according to overthreshold
                angVelIValue_.y = max(angVelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.y < -angVelLimit_.y) {
                angVelIValue_.y -= angVelOutput.y + angVelLimit_.y; //Remove saturation from I according to overthreshold
                angVelIValue_.y = min(angVelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angVelOutput.z > angVelLimit_.z) {
                angVelIValue_.z -= angVelOutput.z - angVelLimit_.z; //Remove saturation from I according to overthreshold
                angVelIValue_.z = max(angVelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angVelOutput.z < -angVelLimit_.z) {
                angVelIValue_.z -= angVelOutput.z + angVelLimit_.z; //Remove saturation from I according to overthreshold
                angVelIValue_.z = min(angVelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            angVelOutput = error.compWiseMulti(angVelPF_) + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) + angVelIValue_; //Recalculate new output

            //Constrain output
            angVelOutput.x = constrain(angVelOutput.x, -angVelLimit_.x, angVelLimit_.x);
            angVelOutput.y = constrain(angVelOutput.y, -angVelLimit_.y, angVelLimit_.y);
            angVelOutput.z = constrain(angVelOutput.z, -angVelLimit_.z, angVelLimit_.z);

            if (angVelPassThrough_) outputTotal += angVelOutput;
            else setpoint.angularAcceleration += angVelOutput;
            

        }

        if (setpoint.attitudeControlMode.accelerationControl) {

            Vector<> error = navigationData.data.attitude.copy().conjugate().rotateVector(setpoint.angularAcceleration - navigationData.data.angularAcceleration); //Calculate setpoint error and then rotate to local coordinate system.

            angAccelIValue_ += error.compWiseMulti(angAccelIF_);

            Vector<> angAccelOutput = error.compWiseMulti(angAccelPF_)/* + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) currently not implemented */ + angAccelIValue_;

            if (angAccelOutput.x > angAccelLimit_.x) {
                angAccelIValue_.x -= angAccelOutput.x - angAccelLimit_.x; //Remove saturation from I according to overthreshold
                angAccelIValue_.x = max(angAccelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.x < -angAccelLimit_.x) {
                angAccelIValue_.x -= angAccelOutput.x + angAccelLimit_.x; //Remove saturation from I according to overthreshold
                angAccelIValue_.x = min(angAccelIValue_.x, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angAccelOutput.y > angAccelLimit_.y) {
                angAccelIValue_.y -= angAccelOutput.y - angAccelLimit_.y; //Remove saturation from I according to overthreshold
                angAccelIValue_.y = max(angAccelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.y < -angAccelLimit_.y) {
                angAccelIValue_.y -= angAccelOutput.y + angAccelLimit_.y; //Remove saturation from I according to overthreshold
                angAccelIValue_.y = min(angAccelIValue_.y, 0.0f); //Make sure not to remove so much that it goes negative
            }
            if (angAccelOutput.z > angAccelLimit_.z) {
                angAccelIValue_.z -= angAccelOutput.z - angAccelLimit_.z; //Remove saturation from I according to overthreshold
                angAccelIValue_.z = max(angAccelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            } else if (angAccelOutput.z < -angAccelLimit_.z) {
                angAccelIValue_.z -= angAccelOutput.z + angAccelLimit_.z; //Remove saturation from I according to overthreshold
                angAccelIValue_.z = min(angAccelIValue_.z, 0.0f); //Make sure not to remove so much that it goes negative
            }

            angAccelOutput = error.compWiseMulti(angAccelPF_)/* + (setpoint.angularAcceleration - navigationData.data.angularAcceleration).compWiseMulti(angVelDF_) currently not implemented */ + angAccelIValue_; //Recalculate new output

            //Constrain output
            angAccelOutput.x = constrain(angAccelOutput.x, -angAccelLimit_.x, angAccelLimit_.x);
            angAccelOutput.y = constrain(angAccelOutput.y, -angAccelLimit_.y, angAccelLimit_.y);
            angAccelOutput.z = constrain(angAccelOutput.z, -angAccelLimit_.z, angAccelLimit_.z);

        }

        controlOutput_.data.torqe = outputTotal;
        //Serial.println(String("Test: x: ") + controlOutput_.data.torqe.x + ", y: " + controlOutput_.data.torqe.y + ", z: " + controlOutput_.data.torqe.z);

    }

    

    //Update control output timestamp
    controlOutput_.timestamp = NOW();

    controlTopic_.publish(controlOutput_);

}
