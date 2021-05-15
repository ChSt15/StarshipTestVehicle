#include "starship_control.h"




void StarshipControl::thread() {

    
    //Attitude control section

    if (controlSetpoint_->attitudeControlMode == eControlMode_t::eControlMode_Disable) {

        controlOutput_.force = 0;
        controlOutput_.torqe = 0;

    } else {    

        controlOutput_.torqe = 0;

        ControlData setpoint = *controlSetpoint_;

        Vector attitudeOutput(0);
        Vector angVelOutput(0);
        Vector angAccelOutput(0);

        Vector outputTotal(0);

        if (setpoint.attitudeControlMode == eControlMode_t::eControlMode_Position || setpoint.attitudeControlMode == eControlMode_t::eControlMode_Velocity_Position || setpoint.attitudeControlMode == eControlMode_t::eControlMode_Acceleration_Velocity_Position) {

            Vector error = (setpoint.attitude*navigationData_->attitude.copy().conjugate()).toVector(); //Error is calculated here already in local coordinate system.

            //Serial.println(String("Error: x: ") + error.x + ", y: " + error.y + ", z: " + error.z);

            attitudeIValue_ += error.compWiseMulti(attitudeIF_);

            Vector attitudeOutput = error.compWiseMulti(attitudePF_) + (setpoint.angularRate - navigationData_->angularRate).compWiseMulti(attitudeDF_) + attitudeIValue_;

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

            attitudeOutput = error.compWiseMulti(attitudePF_) + (setpoint.angularRate - navigationData_->angularRate).compWiseMulti(attitudeDF_) + attitudeIValue_; //Recalculate new output

            //Constrain output
            attitudeOutput.x = constrain(attitudeOutput.x, -attitudeLimit_.x, attitudeLimit_.x);
            attitudeOutput.y = constrain(attitudeOutput.y, -attitudeLimit_.y, attitudeLimit_.y);
            attitudeOutput.z = constrain(attitudeOutput.z, -attitudeLimit_.z, attitudeLimit_.z);

            if (attitudePassThrough_) outputTotal += attitudeOutput;
            else setpoint.angularRate += attitudeOutput;

        }

        if (setpoint.attitudeControlMode == eControlMode_t::eControlMode_Velocity || setpoint.attitudeControlMode == eControlMode_t::eControlMode_Velocity_Position || setpoint.attitudeControlMode == eControlMode_t::eControlMode_Acceleration_Velocity_Position) {

            Vector error = navigationData_->attitude.copy().conjugate().rotateVector(setpoint.angularRate - navigationData_->angularRate); //Calculate setpoint error and then rotate to local coordinate system.

            angVelIValue_ += error.compWiseMulti(angVelIF_);

            Vector angVelOutput = error.compWiseMulti(angVelPF_) + (setpoint.angularAcceleration - navigationData_->angularAcceleration).compWiseMulti(angVelDF_) + angVelIValue_;

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

            angVelOutput = error.compWiseMulti(angVelPF_) + (setpoint.angularAcceleration - navigationData_->angularAcceleration).compWiseMulti(angVelDF_) + angVelIValue_; //Recalculate new output

            //Constrain output
            angVelOutput.x = constrain(angVelOutput.x, -angVelLimit_.x, angVelLimit_.x);
            angVelOutput.y = constrain(angVelOutput.y, -angVelLimit_.y, angVelLimit_.y);
            angVelOutput.z = constrain(angVelOutput.z, -angVelLimit_.z, angVelLimit_.z);

            if (angVelPassThrough_) outputTotal += angVelOutput;
            else setpoint.angularAcceleration += angVelOutput;
            

        }

        if (setpoint.attitudeControlMode == eControlMode_t::eControlMode_Acceleration || setpoint.attitudeControlMode == eControlMode_t::eControlMode_Acceleration_Velocity || setpoint.attitudeControlMode == eControlMode_t::eControlMode_Acceleration_Velocity_Position) {

            Vector error = navigationData_->attitude.copy().conjugate().rotateVector(setpoint.angularAcceleration - navigationData_->angularAcceleration); //Calculate setpoint error and then rotate to local coordinate system.

            angAccelIValue_ += error.compWiseMulti(angAccelIF_);

            Vector angAccelOutput = error.compWiseMulti(angAccelPF_)/* + (setpoint.angularAcceleration - navigationData_->angularAcceleration).compWiseMulti(angVelDF_) currently not implemented */ + angAccelIValue_;

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

            angAccelOutput = error.compWiseMulti(angAccelPF_)/* + (setpoint.angularAcceleration - navigationData_->angularAcceleration).compWiseMulti(angVelDF_) currently not implemented */ + angAccelIValue_; //Recalculate new output

            //Constrain output
            angAccelOutput.x = constrain(angAccelOutput.x, -angAccelLimit_.x, angAccelLimit_.x);
            angAccelOutput.y = constrain(angAccelOutput.y, -angAccelLimit_.y, angAccelLimit_.y);
            angAccelOutput.z = constrain(angAccelOutput.z, -angAccelLimit_.z, angAccelLimit_.z);

        }

        controlOutput_.torqe = outputTotal;
        //Serial.println(String("Test: x: ") + controlOutput_.torqe.x + ", y: " + controlOutput_.torqe.y + ", z: " + controlOutput_.torqe.z);

    }

    controlOutput_.force.x = 0;
    controlOutput_.force.y = 0;
    controlOutput_.force.z = (1.0-navigationData_->position.z)*positionPF_.z + navigationData_->velocity.z*velocityPF_.z + 9.81*1; 
    controlOutput_.force = navigationData_->attitude.copy().conjugate().rotateVector(controlOutput_.force); //Rotate to local coordinate system

    //Update control output timestamp
    controlOutput_.timestamp = micros();



}
