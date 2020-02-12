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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name="Auto Class Based Drive Test", group="Linear Opmode")

public class AutoClassBasedDriveTest extends AutoOP_ClassBased {


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

        if (opModeIsActive()) {
            driveTrain.encoderDrive(15, false);
        }
    }

    public boolean SkyStoneVisible(double timeout) {
        ElapsedTime timey = new ElapsedTime();
        timey.reset();
        while(timey.seconds() < timeout) {
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

    public void hitSkyStone(double inches){
        encoderMovement(9.5 + inches, 3,false);
        driveTrain.encoderTurn(90, inchesPerDegrees, false);
        encoderMovement(-24, 5, false);
        gripper.grabStone(driveTrain);
    }

    public void hitSkyStone2(double inches){
        encoderMovement(9.5 + inches, 3, false);
        driveTrain.encoderTurn(90, inchesPerDegrees, false);
        armShoulder.to(85);
        armElbow.to(-40);
        gripper.openMax();
        armShoulder.to(15);
    }

    public void collectSkyStone(double inches){
        encoderMovement(-2.5 + inches, 2, false);
        driveTrain.encoderTurn(93, inchesPerDegrees, false);
        encoderMovement(-16, 5, false);
        armShoulder.to(120);
        gripper.openMax();

        //need to add in a method that moves shoulder and elbow at the same time, the two lines are what need to be replaced with one
        armShoulder.to(15);
        armElbow.to(-40);

        encoderMovement(12, 4,false);
        armElbow.to(-10);
        armShoulder.to(60);
        encoderMovement(-4, 2, false);
        gripper.closeUntilTouching();

        //need a method that moves elbow and shoulder at same time, next two lines are what need replaced with one line
        armShoulder.to(180);
        armElbow.to(-100);

        encoderMovement(6, 3,false);
        driveTrain.encoderTurn(90, inchesPerDegrees,false);
        encoderMovement(-50, 6, false);
        gripper.openMax();
        encoderMovement(10, 4,  false);
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