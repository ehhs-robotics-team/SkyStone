/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name="IMU Test", group="Linear Opmode")

public class AutoIMU_Test extends AutoOP_ClassBased {
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;



    @Override
    public void main() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //actual auto
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()){
            turnByIMUabsolute(90, 5);
        }
        stopRobot();
    }

    public void turnLeft(double turnAngle, double timeoutS) {
        sleep(500);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed=.5;
        double oldDegreesLeft=turnAngle;
        double scaledSpeed=speed;
        double targetHeading=angles.firstAngle+turnAngle;
        double oldAngle=angles.firstAngle;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}
        double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        runtime.reset();
        while(opModeIsActive() &&
                runtime.seconds() < timeoutS &&
                degreesLeft>1&&
                oldDegreesLeft-degreesLeft>=0) { //check to see if we overshot target
            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
            if(scaledSpeed>1){scaledSpeed=.1;}
            b_leftDrive.setPower(scaledSpeed*1.3); //extra power to back wheels
            b_rightDrive.setPower(-1*scaledSpeed*1.3); //due to extra weight
            f_leftDrive.setPower(scaledSpeed);
            f_rightDrive.setPower(-1*scaledSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft=degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
            if(Math.abs(angles.firstAngle-oldAngle)<1){speed*=1.1;} //bump up speed to wheels in case robot stalls before reaching target
            oldAngle=angles.firstAngle;
        }
        stopRobot(); //our helper method to set all wheel motors to zero
        sleep(250); //small pause at end of turn
    }

    /** Very hypothetical imu turn program based on https://www.youtube.com/watch?v=ZdBbAKsgiQI&feature=youtu.be
     *
     * @param target
     * @param timeOut
     */
    public void turnByIMUabsolute(int target, double timeOut){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        navTime.reset();

        double basePower = 1;
        double power;
        double additionalPower = 0.125;
        double accuracy = 0.3;

        while (opModeIsActive() && Math.abs(currentHeading - target) > accuracy && navTime.seconds() < timeOut){
            if(target<-180) {target+=360;}
            if(target>180){target-=360;}
            if (currentHeading > target){
                power = ((basePower * (currentHeading-target)/100) + additionalPower);
                b_leftDrive.setPower(-1*power);
                b_rightDrive.setPower(power);
                f_leftDrive.setPower(-1*power);
                f_rightDrive.setPower(power);

            }

            if (currentHeading < target){
                power = ((basePower * (target - currentHeading)/100) + additionalPower);

                b_leftDrive.setPower(power);
                b_rightDrive.setPower(-1*power);
                f_leftDrive.setPower(power);
                f_rightDrive.setPower(-1*power);
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = angles.firstAngle;

            telemetry.addData("Current Heading", currentHeading);
            telemetry.update();
        }
        stopRobot();

        telemetry.addData("Current Heading", currentHeading);
        telemetry.update();
    }
}