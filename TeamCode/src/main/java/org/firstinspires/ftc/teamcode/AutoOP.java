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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


/**
 * This file contains the autonomous code for the FTC 15116 Nuclear Nerdz robot
 * "Neil Armstrong" for the 2019-20 Skystone Competition.
 *
 */

@Autonomous(name="Auto Parent", group="Linear Opmode")
@Disabled
public abstract class AutoOP extends LinearOpMode {
    //DRIVE TRAIN MOTOR VARIABLES
    // Declare the motor variables
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime encoderTime = new ElapsedTime();

    private DcMotor f_leftDrive = null;
    private DcMotor f_rightDrive = null;
    private DcMotor b_leftDrive = null;
    private DcMotor b_rightDrive = null;

    //Declare the variables for the claw servos
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    //the gripper servo
    CRServo gripperServo = null;


    //setup sensitivity variables
    double clawUpPosition = 1.0;
    double clawDownPosition = 0.55;

    //Declare encoder variables
    static final double COUNTS_PER_MOTOR_TETRIX = 1440;    // Tetrix Matrix 12V motor with 52.8:1 gearbox
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 2.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = .8;//0.6;
    static final double TURN_SPEED = .7;//0.5;
    public double inchesPerDegrees = 0.06166;


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


    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public VuforiaTrackables targetsSkyStone = null;


    public boolean targetVisible = false;
    public float phoneXRotate = 0;
    public float phoneYRotate = 0;
    public float phoneZRotate = 0;


    //setup the values that are needed
    double drivePower = 0.25;

    //variable to see if turn is completed or not
    public boolean completedTurn = false;

    //setup SIMPLE X, Y, and Z values and heading
    public double xPos = 0;
    public double yPos = 0;
    public double zPos = 0;
    public double currentHeading = 0;

    //slow down rate for encoders
    private double slowDownRate = 0.45;


    //Declare the public translation variable
    public VectorF translation = null;


    // Arm control variables

    public DcMotor armShoulder = null;
    public DcMotor armElbow = null;


    // SEt initial angle to the angle the 1st arm segment is at when resting on the robot (degrees) ;
    final double START_SHOULDER_ANGLE = -12;
    double currentShoulderAngle = START_SHOULDER_ANGLE;

    // SEt initial angle to the angle the 2nd arm segment is at when raesting on the robot (degrees) ;
    double START_ELBOW_ANGLE = 160 - START_SHOULDER_ANGLE;
    double currentElbowAngle = START_ELBOW_ANGLE;

    final double SHOULDER_TICKS_PER_ROTATION = 1440;
    final double ELBOW_TICKS_PER_ROTATION = 1120; // Rev motor as per http://www.revrobotics.com/content/docs/Encoder-Guide.pdf

    final double SHOULDER_GEAR_RATIO = 1.0 / 3.0; // Motor:Shoulder Motor turns 3 times per one arm rotation
    final double ELBOW_GEAR_RATIO = 3.0 / 8.0; // Motor:Elbow gear ratio

    final double SHOULDER_TICKS_PER_DEGREE = SHOULDER_TICKS_PER_ROTATION / 360;
    final double ELBOW_TICKS_PER_DEGREE = ELBOW_TICKS_PER_ROTATION / 360;

    // Aid at the extremities, to keep the arm still at full horizontal extension.
    double MAX_SHOULDER_AID = 0.002;
    double MAX_ELBOW_AID = 0.0005;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {


        //DRIVE TRAIN MOTOR SETUP
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        f_leftDrive = hardwareMap.get(DcMotor.class, "f_leftDrive");
        f_rightDrive = hardwareMap.get(DcMotor.class, "f_rightDrive");
        b_leftDrive = hardwareMap.get(DcMotor.class, "b_leftDrive");
        b_rightDrive = hardwareMap.get(DcMotor.class, "b_rightDrive");

        //reverse the right motors
        f_leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        b_leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        f_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        b_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        f_leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b_leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        f_rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b_rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //map the claw servos
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");


        //map the gripper's servo
        gripperServo = hardwareMap.get(CRServo.class, "gripperServo");

        //reverse one of the claw servos
        rightClaw.setDirection(Servo.Direction.REVERSE);

        // Init arm variables

        armElbow = hardwareMap.get(DcMotor.class, "arm_elbow");
        armShoulder = hardwareMap.get(DcMotor.class, "arm_shoulder");

        armShoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        armReset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Run the child autonomous run
        main();

    }


    /*
     * method for children autonomous opmodes to override and insert case specific moves.
     */
    public abstract void main();


    ElapsedTime navTime = new ElapsedTime();

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



                attempt to move based on vuforia position
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

    public void forward() {
        f_leftDrive.setPower(drivePower);
        b_leftDrive.setPower(drivePower);
        f_rightDrive.setPower(drivePower);
        b_rightDrive.setPower(drivePower);
    }

    public void encoderLinear(double inches, double timeout, boolean slowDown) {
        if (!slowDown) {
            encoderDrive(DRIVE_SPEED, inches, inches, timeout);
        } else if (slowDown) {
            encoderDrive(slowDownRate, inches, inches, timeout);
        }
    }

    public void encoderLinear(double inches, double timeout) {
        encoderLinear(inches, timeout, true);
    }


    public void backward() {
        f_leftDrive.setPower(-drivePower);
        b_leftDrive.setPower(-drivePower);
        f_rightDrive.setPower(-drivePower);
        b_rightDrive.setPower(-drivePower);
    }

    public void left() {
        f_leftDrive.setPower(drivePower);
        b_leftDrive.setPower(drivePower);
        f_rightDrive.setPower(-drivePower);
        b_rightDrive.setPower(-drivePower);
    }

    public void right() {
        f_leftDrive.setPower(-drivePower);
        b_leftDrive.setPower(-drivePower);
        f_rightDrive.setPower(drivePower);
        b_rightDrive.setPower(drivePower);
    }

    public void stopRobot() {
        f_leftDrive.setPower(0);
        b_leftDrive.setPower(0);
        f_rightDrive.setPower(0);
        b_rightDrive.setPower(0);
        telemetry.addData("Ended braking at :", "X:" + xPos + " Y:" + yPos + " Z:" + zPos);
        telemetry.update();
    }


    //Control methods for the claw
    public void clawDown(double position){
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }

    public void clawDown(){
        leftClaw.setPosition(clawDownPosition);
        rightClaw.setPosition(clawUpPosition);
    }

    public void clawUp(){
        rightClaw.setPosition(clawUpPosition);
        leftClaw.setPosition(clawUpPosition);
    }

    public void initVuforia(){
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);


        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = -90;

        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
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
            if (speedD == DRIVE_SPEED){
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


    /* Method to calculate the power added to the shoulder to keep it stationary during play.
     * Employs cosine to keep max power at horizontal position and zero power at vertical.
     */
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

    /* Method to calculate the power added to the elbow to keep it stationary during play.
     * Automatically adjusts aid for the varying position of the shoulder.
     * Employs cosine to keep max power at horizontal position and zero power at vertical.
     */
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

    /*  Method to perform a relative move of the shoulder section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderShoulder(double speed,
                                double degrees,
                                double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = armShoulder.getCurrentPosition() + (int) (degrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
            //newTarget *= 2;
            armShoulder.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            encoderTime.reset();
            armShoulder.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (encoderTime.seconds() < timeoutS) &&
                    (armShoulder.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTarget);
                telemetry.addData("Path2", "Running at %7d",
                        armShoulder.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            armShoulder.setPower(0);

            // Turn off RUN_TO_POSITION
            armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    /*  Method to perform a relative move of the shoulder section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     *  tolerance: number of encoder ticks away from the target the method will accept as accurate.
     */
    public void encoderShoulder(double speed,double degrees,
                                double tolerance, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = armShoulder.getCurrentPosition() + (int) (degrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
            //newTarget *= 2;
            armShoulder.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            encoderTime.reset();
            armShoulder.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (encoderTime.seconds() < timeoutS) &&
                    (armShoulder.isBusy()) &&
                    (Math.abs(armShoulder.getCurrentPosition()- armShoulder.getTargetPosition()) > tolerance)) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newTarget);
                telemetry.addData("Path2", "Running at %7d",
                        armShoulder.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            armShoulder.setPower(0);

            // Turn off RUN_TO_POSITION
            armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    /*  Method to perform a relative move of the elbow section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderElbow(double speed, double degrees, double timeoutS){
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = armElbow.getCurrentPosition() + (int) (-degrees * ELBOW_TICKS_PER_DEGREE/ELBOW_GEAR_RATIO);
            newTarget *= 2;
            armElbow.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            encoderTime.reset();
            armElbow.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (encoderTime.seconds() < timeoutS) &&
                    (armElbow.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Elbow", "Running at %7d to %7d",
                        armElbow.getCurrentPosition(), newTarget);
                telemetry.update();
            }

            // Stop all motion;
            armElbow.setPower(calculateElbowAid());

            // Turn off RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    /*  Method to perform a relative move of the elbow section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderArm(double speed, double shoulderDegrees, double elbowDegrees, double timeoutS){
        int newShoulderTarget;
        int newElbowTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newElbowTarget = armElbow.getCurrentPosition() + (int) (-elbowDegrees * ELBOW_TICKS_PER_DEGREE/ELBOW_GEAR_RATIO);
            newElbowTarget *= 2;
            armElbow.setTargetPosition(newElbowTarget);

            // Determine new target position, and pass to motor controller
            newShoulderTarget = armShoulder.getCurrentPosition() + (int) (shoulderDegrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
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


    public void grabStone(){
        openGripper(2.0);
        encoderTurn(5, 5);
        closeGripper(1.0);
        encoderTurn(-5,5);
    }


    public void openGripper(double seconds){
       if(opModeIsActive()) {
           gripperServo.setPower(1);
           sleep((long)(seconds * 1000));
           gripperServo.setPower(0);
       }
    }

    public void openGripper(long milliseconds){
        if(opModeIsActive()) {
            gripperServo.setPower(1);
            sleep(milliseconds);
            gripperServo.setPower(0);
        }
    }


    public void closeGripper(double seconds) {
        if (opModeIsActive()) {
            gripperServo.setPower(-1);
            sleep((long) (seconds * 1000));
            gripperServo.setPower(0);
        }
    }
}












