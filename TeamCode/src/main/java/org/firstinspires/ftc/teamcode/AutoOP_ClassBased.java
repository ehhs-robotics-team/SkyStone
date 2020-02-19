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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Parent TeleOP class to hold driver controls.
 */


@Autonomous(name="TeleOP Parent Class", group ="Linear Opmode")
@Disabled
public abstract class AutoOP_ClassBased extends LinearOpMode {
    // Timer variables
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime navTime = new ElapsedTime();
    public ElapsedTime encoderTime = new ElapsedTime();

    //DRIVE TRAIN MOTOR VARIABLES
    DcMotor f_leftDrive = null;
    DcMotor f_rightDrive = null;
    DcMotor b_leftDrive = null;
    DcMotor b_rightDrive = null;

    public Motor armShoulder;
    public Motor armElbow;
    public Gripper gripper;
    public Claw claw;
    public DriveTrain driveTrain;

    //IMU sensor variables
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //Declare drive encoder variables
    static final double COUNTS_PER_MOTOR_TETRIX = 1440;    // Tetrix Matrix 12V motor with 52.8:1 gearbox
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 2.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = .8;//0.6;
    static final double TURN_SPEED = .7;//0.5;
    static final double inchesPerDegrees = 0.06166;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AbEiBY3/////AAAAGbH/Sq+b9ULwoWFe7pLZlcRNDVckrjct+bi1aw5bYvqmY0YPNOfIdPK19cMBDwdyeIMLZ202x5VD0rmxkGWLlVXocop6qzZXp1bbQQMVVKUdXaPOvqnfvbfC9EhJ+Cy9digZVz+F2Cffvm9zZ9RBLIjb3O4i8+b3qBGk3NWQNQYdHLt4f7t9QlsOdU1yyvBTAxvxa7yIzWGlmZHAdbBZpETCiIwaSG7Ykn17FokNPOGHcQ9QvERwUTbp92azytukPOnHRNW2IltM8kd1GFMqMASAii14EIIRvDtqEiQmWhHE0/5qgRmpkK0ZovmgPSRQCg4AOIRUGbWqDTvhIXqAaXtRinO5/Itt9yOZnBLvz0mK";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;


    // Class Members
    public OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    //Declare the public translation variable
    public VectorF translation = null;

    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public VuforiaTrackables targetsSkyStone = null;


    public boolean targetVisible = false;
    public float phoneXRotate = 0;
    public float phoneYRotate = 0;
    public float phoneZRotate = 0;

    //setup SIMPLE X, Y, and Z values and heading
    public double xPos = 0;
    public double yPos = 0;
    public double zPos = 0;
    public double currentHeading = 0;
    //variable to see if turn is completed or not
    public boolean completedTurn = false;


    @Override
    public void runOpMode() {


        //DRIVE TRAIN MOTOR SETUP
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //map the motors
        f_leftDrive = hardwareMap.get(DcMotor.class, "f_leftDrive");
        f_rightDrive = hardwareMap.get(DcMotor.class, "f_rightDrive");
        b_leftDrive = hardwareMap.get(DcMotor.class, "b_leftDrive");
        b_rightDrive = hardwareMap.get(DcMotor.class, "b_rightDrive");


        //reverse the right motors
        f_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        b_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        //map the motors
        //map the motors and set default running values
        armShoulder = new Motor(hardwareMap, "arm_shoulder",
                -12, 1440, 1.0/10.0, 0,
                DcMotorSimple.Direction.REVERSE, this);


        armElbow = new Motor(hardwareMap, "arm_elbow",
                180, 1120, 3.0/8.0, 0.0005,
                DcMotorSimple.Direction.FORWARD, this);

        gripper = new Gripper(hardwareMap, "gripperMotor",
                0, 1440, 3.5, 0,
                DcMotor.Direction.FORWARD, this);

        claw = new Claw(hardwareMap, "rightClaw", "leftClaw", this);

        driveTrain = new DriveTrain(f_leftDrive, f_rightDrive, b_leftDrive, b_rightDrive, this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Run the child teleop run
        main();


    }

    /*
     * method for children autonomous opmodes to override and insert case specific moves.
     */
    public abstract void main();


    public void navigateToHeading(double targetHeading, double encoderTimeout, double vuforiaTimeout) {
        navTime.reset();
        while (!isStopRequested() && navTime.seconds() < vuforiaTimeout) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;

                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                //Continue to setup the variables to get the position of the robot
                translation = lastLocation.getTranslation();
                xPos = translation.get(0) / mmPerInch;
                yPos = translation.get(1) / mmPerInch;
                zPos = translation.get(2) / mmPerInch;
                currentHeading = rotation.thirdAngle;

                turnToHeading(targetHeading, encoderTimeout);
                stopRobot();
                break;


            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

            /*if (targetVisible) {
                // attempt to move based on vuforia position
                if (xPos > -20) {
                    forward();
                } else if (xPos < -20){
                    telemetry.addData("Attempting to brake at:", "X:" + xPos + " Y:" + yPos + " Z:" + zPos);
                    telemetry.update();
                    stopRobot();
                }

            }
            */
        }
    }

    //Functions for simple movement of the robot
    //calculates the angle we need to turn the robot to a heading
    public double angleFromPosition(double targetX, double targetY) {
        double deltaY = targetY - yPos;
        double deltaX = targetX - xPos;
        double angle360 = (Math.atan(deltaY / deltaX));
        if (angle360 <= 180) {
            return angle360;
        } else {
            return (angle360 - 360);
        }
    }


    //turns the robot to a heading given a heading (must be a value between -180 and 180).
    // Returns true if the robot has completed the turn, and returns false if the robot is still turning
    public void turnToHeading(double angle, double timeout) {
        if (Math.abs(currentHeading - angle) < 10) {
            stopRobot();
            completedTurn = true;
        } else {
            if (angle > currentHeading) {
                encoderTurn(currentHeading - angle, timeout);
            }
            if (currentHeading > angle) {
                encoderTurn(currentHeading - angle, timeout);
            }
        }

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speedD, double leftInches, double rightInches, double timeoutS) {

        int newf_RightTarget;
        int newf_LeftTarget;
        int newb_RightTarget;
        int newb_LeftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //divide the drive inches by two for more accurate movement
            if (speedD == DRIVE_SPEED) {
                leftInches /= 2;
                rightInches /= 2;
            }

            // Determine new target position, and pass to motor controller
            newf_RightTarget = b_rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newf_LeftTarget = b_leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newb_RightTarget = f_rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newb_LeftTarget = f_leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            b_leftDrive.setTargetPosition(newf_LeftTarget);
            b_rightDrive.setTargetPosition(newf_RightTarget);
            f_leftDrive.setTargetPosition(newb_LeftTarget);
            f_rightDrive.setTargetPosition(newb_RightTarget);

            // Turn On RUN_TO_POSITION
            b_leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            b_rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            f_leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            f_rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            b_leftDrive.setPower(Math.abs(speedD));
            b_rightDrive.setPower(Math.abs(speedD));
            f_leftDrive.setPower(Math.abs(speedD));
            f_rightDrive.setPower(Math.abs(speedD));

            // keep looping while we are still active, and there is time left, and all motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when ANY motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that ALL motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (b_leftDrive.isBusy() && b_rightDrive.isBusy() && f_leftDrive.isBusy() && f_leftDrive.isBusy())) {
                // Display data for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newf_LeftTarget, newf_RightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        b_leftDrive.getCurrentPosition(),
                        b_rightDrive.getCurrentPosition(),
                        f_leftDrive.getCurrentPosition(),
                        f_rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            f_leftDrive.setPower(0);
            f_rightDrive.setPower(0);
            b_leftDrive.setPower(0);
            b_rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            f_leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            f_rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            b_leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            b_rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        telemetry.addData("Drive status: ", "complete");
        telemetry.update();
    }

    //a method to turn a certain amount of degrees with encoders
    public void encoderTurn(double degrees, double timeout) {
        encoderDrive(TURN_SPEED, (-degrees * inchesPerDegrees * 2), (degrees * inchesPerDegrees * 2), timeout);
    }

    public void encoderTurn(double degrees, double speed, double timeout) {
        encoderDrive(speed, (-degrees * inchesPerDegrees * 2), (degrees * inchesPerDegrees * 2), timeout);
    }

    public void stopRobot() {
        f_leftDrive.setPower(0);
        b_leftDrive.setPower(0);
        f_rightDrive.setPower(0);
        b_rightDrive.setPower(0);
        telemetry.addData("Ended braking at :", "X:" + xPos + " Y:" + yPos + " Z:" + zPos);
        telemetry.update();
    }

    // reset the arm function during play to account for slippage.
    public void armReset() {
        armElbow.reset();
        armShoulder.reset();
    }

    /*  Method to perform an absolute move of the elbow section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void armTo(double speed, double shoulderDegrees, double elbowDegrees, double timeoutS) {
        armShoulder.encoderMode();
        armElbow.encoderMode();
        encoderTime.reset();

        while (opModeIsActive() &&
                (encoderTime.seconds() < timeoutS) &&
                (armElbow.motor.isBusy() || armShoulder.motor.isBusy())) {
            armShoulder.to((int) shoulderDegrees);
            armElbow.to((int) elbowDegrees);
        }

        armShoulder.powerMode();
        armElbow.powerMode();
        // Aid the elbow section to keep it from falling;
        armElbow.setPower(armElbow.calculateAid(armShoulder.motor.getCurrentPosition(), telemetry));
    }

    public void encoderMovement(double inches, double timeout, boolean slowMode){
        encoderTime.reset();
        while (opModeIsActive() && timeout < encoderTime.seconds()){
            driveTrain.encoderDrive(inches, slowMode);
        }
    }



    public void closeGripper(double timeout){
        runtime.reset();
        if (opModeIsActive()){
            gripper.closeUntilTouching();
            while (opModeIsActive() && runtime.seconds() < timeout && !gripper.isTouching()){
                ;
            }

            //stop all motion (yeah)
            gripper.setPower(0);
        }

    }

    //overloaded version of close gripper with default timeout value that we acquired from TESTING
    public void closeGripper() {
        closeGripper(1.2);
    }


    //method to open the gripper
    public void openGripper(double timeout){
        runtime.reset();
        if (opModeIsActive()){
            gripper.openMax();
            while (opModeIsActive() && runtime.seconds() < timeout && gripper.motor.isBusy()){
                ;
            }

            //stop all motion (yeah)
            gripper.setPower(0);
        }
    }

    //overloaded version of open gripper with default timeout value that we acquired from TESTING
    public void openGripper(){
        openGripper(1.2);
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
