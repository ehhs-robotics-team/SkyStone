/* Copyright (c) 2019 FIRST. All rights reserved.
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

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Parent TeleOP class to hold driver controls.
 */


@TeleOp(name="Tele Parent", group ="Linear Opmode")
@Disabled
public abstract class TeleOP extends LinearOpMode {

    //DRIVE TRAIN MOTOR VARIABLES
    // Declare the motor variables
    ElapsedTime runtime = new ElapsedTime();

    public ElapsedTime encoderTime = new ElapsedTime();
    DcMotor f_leftDrive = null;
    DcMotor f_rightDrive = null;
    DcMotor b_leftDrive = null;
    DcMotor b_rightDrive = null;

    public TouchSensor touchy;

    DcMotor armShoulder = null;
    DcMotor armElbow = null;

    //Declare the variables for the claw servos
    Servo leftClaw = null;
    Servo rightClaw = null;

    //Declare the gripper's servo variable
    public DcMotor gripperMotor;

    double driveSensitivity;
    double clawUpPosition;
    double clawDownPosition;

    //flag variables
    public static boolean started = false;



    // SEt initial angle to the angle the 1st arm segment is at when resting on the robot (degrees) ;
    public final double START_SHOULDER_ANGLE = -12;
    double currentShoulderAngle = START_SHOULDER_ANGLE;

    // SEt initial angle to the angle the 2nd arm segment is at when raesting on the robot (degrees) ;
    public double START_ELBOW_ANGLE = 160-START_SHOULDER_ANGLE;
    double currentElbowAngle = START_ELBOW_ANGLE;

    final double GRIPPER_TICKS_PER_ROTATION = 1440;
    final double SHOULDER_TICKS_PER_ROTATION = 1440;
    final double ELBOW_TICKS_PER_ROTATION = 1120; // Rev motor as per http://www.revrobotics.com/content/docs/Encoder-Guide.pdf

    final double SHOULDER_GEAR_RATIO = 1.0/10.0; // Motor:Shoulder Motor turns 3 times per one arm rotation
    final double ELBOW_GEAR_RATIO = 3.0/8.0; // Motor:Elbow gear ratio

    final double GRIPPER_TICKS_PER_INCH = (GRIPPER_TICKS_PER_ROTATION/3.5);
    final double SHOULDER_TICKS_PER_DEGREE = SHOULDER_TICKS_PER_ROTATION/360;
    final double ELBOW_TICKS_PER_DEGREE = ELBOW_TICKS_PER_ROTATION / 360;

    // Aid at the extremities, to keep the arm still at full horizontal extension.
    double MAX_SHOULDER_AID = 0.001;
    double MAX_ELBOW_AID = 0.0005;



    @Override public void runOpMode() {


        //DRIVE TRAIN MOTOR SETUP
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //map the motors
        f_leftDrive  = hardwareMap.get(DcMotor.class, "f_leftDrive");
        f_rightDrive = hardwareMap.get(DcMotor.class, "f_rightDrive");
        b_leftDrive = hardwareMap.get(DcMotor.class, "b_leftDrive");
        b_rightDrive = hardwareMap.get(DcMotor.class, "b_rightDrive");

        armElbow = hardwareMap.get(DcMotor.class, "arm_elbow");
        armShoulder = hardwareMap.get(DcMotor.class, "arm_shoulder");

        //map the claw servos
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        //map the gripper's servo
        gripperMotor = hardwareMap.get(DcMotor.class, "gripperMotor");

        //reverse the right motors
        f_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        b_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        armShoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        armElbow.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset arm and gripper encoder counts for accurate tracking;
        gripperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverse one of the claw servos
        rightClaw.setDirection(Servo.Direction.REVERSE);

        //setup the touch sensor
        touchy = hardwareMap.get(TouchSensor.class, "touch");


        //setup the values that are needed
        double driveSensitivity = 1.5;
        double clawUpPosition = 1.0;
        double clawDownPosition = 0.25;




        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Run the child teleop run
        main();


    }

    public double calculateShoulderAid(){
        // Shoulder joint controls
        //telemetry.addData("Shoulder Posisiton: ", armShoulder.getCurrentPosition());

        // Use the position of the encoder and the known starting position of the arm to
        // determine the angle of the 1st arm segment.
        currentShoulderAngle = (armShoulder.getCurrentPosition()/SHOULDER_TICKS_PER_DEGREE)*SHOULDER_GEAR_RATIO;

        // Gets the absolute positioning of the 1st arm segment, assuming it always starts from
        // the "down" position beside the phone mount at START_SHOULDER_ANGLE.
        double adjustedShoulderAngle = currentShoulderAngle + START_SHOULDER_ANGLE;
        telemetry.addData("Shoulder Angle: ", adjustedShoulderAngle );

        // Uses cosine to determine the appropriate aid to add to the arm to hold it stationary:
            /*        _- 0 - _
                    /         \
                 -max         max
                    \         /
                       - 0 -             */
        double shoulderAid = MAX_SHOULDER_AID * Math.cos(Math.toRadians(adjustedShoulderAngle));
        telemetry.addData("Shoulder Aid: ", shoulderAid);

        return shoulderAid;
    }

    public double calculateElbowAid(){
        //Elbow joint controls
        //telemetry.addData("Elbow Posisiton: ", armElbow.getCurrentPosition());

        // Three things determine the angle of the second arm segment.
        // 1. position of the encoder
        // 2. known starting position of the arm
        // 3. The angle of the origin (angle of the 1st arm segment)
        currentElbowAngle = (armElbow.getCurrentPosition()/ELBOW_TICKS_PER_DEGREE)*ELBOW_GEAR_RATIO;
        double adjustedElbowAngle = -currentElbowAngle + START_ELBOW_ANGLE +currentShoulderAngle;
        telemetry.addData("Elbow Angle: ", adjustedElbowAngle);

        // Uses cosine to determine aid using same logic as first segment
        double elbowAid = MAX_ELBOW_AID * Math.cos(Math.toRadians(adjustedElbowAngle));
        telemetry.addData("Elbow Aid: ", elbowAid);


        return elbowAid;
    }

    /*  Method to perform an absolute move of the elbow section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void armTo(double speed, double shoulderDegrees, double elbowDegrees, double timeoutS){
        int newShoulderTarget;
        int newElbowTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newElbowTarget = (int) (-elbowDegrees * ELBOW_TICKS_PER_DEGREE/ELBOW_GEAR_RATIO);
            newElbowTarget *= 2;
            armElbow.setTargetPosition(newElbowTarget);

            // Determine new target position, and pass to motor controller
            newShoulderTarget = + (int) (shoulderDegrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
            // newTarget *= 2;
            armShoulder.setTargetPosition(newShoulderTarget);

            // Turn On RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            encoderTime.reset();
            armElbow.setPower(Math.abs(speed));
            armShoulder.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (encoderTime.seconds() < timeoutS) &&
                    (armElbow.isBusy() || armShoulder.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Elbow", "Running at %7d to %7d",
                        armElbow.getCurrentPosition(), newElbowTarget);
                telemetry.addData("Shoulder", "Running at %7d to %7d",
                        armShoulder.getCurrentPosition(), newElbowTarget);
                telemetry.update();
            }

            // Stop all motion;
            armElbow.setPower(calculateElbowAid());
            armShoulder.setPower(0);

            // Turn off RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }


    /*  Method to perform an absolute move of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void armToContinuous(double speed, double shoulderDegrees, double elbowDegrees){
        int newShoulderTarget;
        int newElbowTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newElbowTarget = (int) (-elbowDegrees * ELBOW_TICKS_PER_DEGREE / ELBOW_GEAR_RATIO);
            newElbowTarget *= 2;
            armElbow.setTargetPosition(newElbowTarget);

            // Determine new target position, and pass to motor controller
            newShoulderTarget = +(int) (shoulderDegrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);

            // If arm is not already running autonomously
            if (armShoulder.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
                    armElbow.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {

                /*
                // Determine new target position, and pass to motor controller
                newElbowTarget = (int) (-elbowDegrees * ELBOW_TICKS_PER_DEGREE / ELBOW_GEAR_RATIO);
                newElbowTarget *= 2;
                armElbow.setTargetPosition(newElbowTarget);

                // Determine new target position, and pass to motor controller
                newShoulderTarget = +(int) (shoulderDegrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
                // newTarget *= 2;
                armShoulder.setTargetPosition(newShoulderTarget);

                 */

                armElbow.setTargetPosition(newElbowTarget);
                armShoulder.setTargetPosition(newShoulderTarget);

                // Turn On RUN_TO_POSITION
                armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // start motion.

                armElbow.setPower(Math.abs(speed));
                armShoulder.setPower(Math.abs(speed));
            }
            else if (Math.abs(armElbow.getCurrentPosition() - newElbowTarget) < 40 &&
                    Math.abs(armShoulder.getCurrentPosition() - newShoulderTarget) < 40 ) {
                stopContinuousArm();
                telemetry.addData("Arm Status:", "Auto Mode turned off");
            } //else if (armElbow.isBusy() || armShoulder.isBusy()) {
            else {
                // Do Nothing or ...

                // Display it for the driver.
                //telemetry.addData("Elbow", "Running at %7d to %7d",
                        //armElbow.getCurrentPosition(), newElbowTarget);
                //telemetry.addData("Shoulder", "Running at %7d to %7d",
                        //armShoulder.getCurrentPosition(), newElbowTarget);
                //telemetry.update();
            }
        }
    }

    /*  Method to perform an absolute move of the gripper section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void gripperTo(double speed, double inches, boolean test, double timeoutS){
        // Ensure that the opmode is still active
        if (!started) {
            if (opModeIsActive()) {
                started = true;
                int newGripperTarget;
                gripperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Determine new target position, and pass to motor controller
                if (test) {
                    newGripperTarget = (int) GRIPPER_TICKS_PER_ROTATION;
                } else {
                    newGripperTarget = (int) (inches * GRIPPER_TICKS_PER_INCH);
                }
                gripperMotor.setTargetPosition(newGripperTarget);

                // Turn On RUN_TO_POSITION
                gripperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                encoderTime.reset();
                gripperMotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and the motors is running.
                while (opModeIsActive() &&
                        (encoderTime.seconds() < timeoutS) &&
                        (gripperMotor.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Gripper", "Running at %7d to %7d",
                            gripperMotor.getCurrentPosition(), newGripperTarget);
                    telemetry.update();
                }

                // Stop all motion;
                gripperMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                gripperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                started = false;
            }
        }

    }

    public void stopContinuousArm(){
        // Stop all motion;
        armElbow.setPower(calculateElbowAid());
        armShoulder.setPower(0);

        // Turn off RUN_TO_POSITION
        armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void clawDown(){
        leftClaw.setPosition(clawDownPosition);
        rightClaw.setPosition(clawUpPosition);
    }
    public void clawDown(double position){
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }

    public void clawUp(){
        rightClaw.setPosition(clawUpPosition);
        leftClaw.setPosition(clawUpPosition);
    }

    public void clawUp(double position){
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }

    // reset the arm function during play to account for slippage.
    public void armReset(){
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // SEt initial angle to the angle the 1st arm segment is at when resting on the robot (degrees) ;
        currentShoulderAngle = START_SHOULDER_ANGLE;

        // SEt initial angle to the angle the 2nd arm segment is at when raesting on the robot (degrees) ;
        currentElbowAngle = START_ELBOW_ANGLE;
    }


    /*
     * method for children autonomous opmodes to override and insert case specific moves.
     */
    public abstract void main();

}
