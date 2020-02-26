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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name="Blue SkyStone 2", group="Linear Opmode")

public class AutoBlueSkystoneMethod2 extends AutoOP_ClassBased {


    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    //boolean to see if we can see the SkyStone or not
    private boolean skystoneVisible = false;
    private boolean skystoneFound = false;
    private Recognition rec = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private static final String VUFORIA_KEY =
            "AbEiBY3/////AAAAGbH/Sq+b9ULwoWFe7pLZlcRNDVckrjct+bi1aw5bYvqmY0YPNOfIdPK19cMBDwdyeIMLZ202x5VD0rmxkGWLlVXocop6qzZXp1bbQQMVVKUdXaPOvqnfvbfC9EhJ+Cy9digZVz+F2Cffvm9zZ9RBLIjb3O4i8+b3qBGk3NWQNQYdHLt4f7t9QlsOdU1yyvBTAxvxa7yIzWGlmZHAdbBZpETCiIwaSG7Ykn17FokNPOGHcQ9QvERwUTbp92azytukPOnHRNW2IltM8kd1GFMqMASAii14EIIRvDtqEiQmWhHE0/5qgRmpkK0ZovmgPSRQCg4AOIRUGbWqDTvhIXqAaXtRinO5/Itt9yOZnBLvz0mK";


    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void main() {
        iniVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

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

        gripper.setClosedPosition(1150);
        gripper.setOpenPosition(-2100);



        //actual auto
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()){
            int position = getSkystonePosition(5);

            // For debugging, to tell what position and at what angle the skystone is detected
            // Bella, make sure getSkystonePosition is returning the correct position.
            telemetry.addData("Skystone Position: ", position);
            telemetry.update();

            tfod.shutdown();

            // Change offset to the value from the center of the robot to the center of the block at position 0;
            double offset = 3;//1.5;
            double inches = (8.5 * position) - offset;
            encoderDrive(0.4, inches, inches, 5);
            sleep(1000);
            turnByIMUabsolute(92, 5);
            armElbow.timedTo(0, 10, armShoulder.getCurrentAngle());
            gripper.openGripper(3);
            encoderDrive(.4, 16,16,5);
            //sleep(3000);
            armElbow.timedTo(-15, 3, armShoulder.getCurrentAngle());
            encoderDrive(.4, 8,8,5);



            gripper.closeGripper(3);
            if (position == 0){
                encoderDrive(0.5, -12,-12,2);
                encoderTurn(20,2);
            } else if (position == 1){
                encoderDrive(0.5, -9, -9, 2);
            } else if (position == 2){
                encoderDrive(0.5, -9, -9, 2);
            }
            elbowTo(10, 2);
            encoderTurn(-110, 5);
            encoderDrive(0.5, 48+inches, 48+inches, 5);
            gripper.openGripper();

            armElbow.to(90);
            gripper.closeGripper(2);
            armElbow.to(170);
            sleep (1000);

            encoderDrive(0.5, -24, -24, 4);






        }

        while (opModeIsActive()){
            gripper.motor.setPower(0);
            telemetry.addData("Autonoumous: ", "finished");
        }
    }

    /**
     * Returns the position (0-2) of the skystone based on the angle from the phone view.
     * Position 0 is at the end, followed by position 1 and 2
     * This method assumes the robot is positioned on the wall with the front wheel aligned to the
     * joint in the field wall and the camera facing the skystones.
     * Currently it only works on the RED side.
     */
    public int getSkystonePosition(double timeout){
        int position = 0;
        double angle = 0;
        if (SkyStoneVisible(timeout)) {
            angle = rec.estimateAngleToObject(AngleUnit.DEGREES);
        } else {
            // Default skystone index if none is detected.
            return 1;
        }
        telemetry.addData("Angle: ", angle);
        if (angle > 10){
            position = 2;
        } else if (angle < 10 && angle > 1) {
            position = 1;
        }
        return position;

    }

    public void elbowTo(double degrees, double timeout){
        encoderTime.reset();
        armElbow.to((int)degrees);
        while (opModeIsActive() && encoderTime.seconds() < timeout && armElbow.motor.isBusy()){
            ;
        }
        armElbow.setPower(armElbow.calculateAid(armShoulder.getCurrentAngle(),telemetry));
    }



    public boolean SkyStoneVisible(double timeout) {
        ElapsedTime timey = new ElapsedTime();
        timey.reset();
        while (timey.seconds() < timeout) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        rec = recognition;
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        float centerPos = recognition.getRight() - recognition.getLeft();


                        if (recognition.getLabel().equals("Skystone")) {
                            skystoneFound = true;
                            return true;
                        }
                    }

                    telemetry.update();

                }
            }


        }

        return false;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void iniVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}