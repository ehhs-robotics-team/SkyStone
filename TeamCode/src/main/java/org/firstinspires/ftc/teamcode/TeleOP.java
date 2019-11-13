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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    DcMotor f_leftDrive = null;
    DcMotor f_rightDrive = null;
    DcMotor b_leftDrive = null;
    DcMotor b_rightDrive = null;

    DcMotor armShoulder = null;
    DcMotor armElbow = null;

    //Declare the variables for the claw servos
    Servo leftClaw = null;
    Servo rightClaw = null;

    //Declare the gripper's servo variable
    CRServo gripperServo = null;

    double driveSensitivity;
    double clawUpPosition;
    double clawDownPosition;



    // SEt initial angle to the angle the 1st arm segment is at when resting on the robot (degrees) ;
    final double START_SHOULDER_ANGLE = -12;
    double currentShoulderAngle = START_SHOULDER_ANGLE;

    // SEt initial angle to the angle the 2nd arm segment is at when raesting on the robot (degrees) ;
    double START_ELBOW_ANGLE = 160-START_SHOULDER_ANGLE;
    double currentElbowAngle = START_ELBOW_ANGLE;


    final double SHOULDER_TICKS_PER_ROTATION = 1440;
    final double ELBOW_TICKS_PER_ROTATION = 1120; // Rev motor as per http://www.revrobotics.com/content/docs/Encoder-Guide.pdf

    final double SHOULDER_GEAR_RATIO = 1.0/3.0; // Motor:Shoulder Motor turns 3 times per one arm rotation
    final double ELBOW_GEAR_RATIO = 3.0/8.0; // Motor:Elbow gear ratio

    final double SHOULDER_TICKS_PER_DEGREE = SHOULDER_TICKS_PER_ROTATION/360;
    final double ELBOW_TICKS_PER_DEGREE = ELBOW_TICKS_PER_ROTATION / 360;

    // Aid at the extremities, to keep the arm still at full horizontal extension.
    final double MAX_SHOULDER_AID = 0.2;
    final double MAX_ELBOW_AID = 0.1;




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
        gripperServo = hardwareMap.get(CRServo.class, "gripperServo");

        //reverse the right motors
        f_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        b_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        armShoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset arm encoder counts for accurate tracking;
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverse one of the claw servos
        rightClaw.setDirection(Servo.Direction.REVERSE);




        //setup the values that are needed
        double driveSensitivity = 1.5;
        double clawUpPosition = 1.0;
        double clawDownPosition = 0.55;




        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Run the child teleop run
        main();

    }

    public double calculateShoulderAid(){
        // Shoulder joint controls
        telemetry.addData("Shoulder Posisiton: ", armShoulder.getCurrentPosition());

        // Use the position of the encoder and the known starting position of the arm to
        // determine the angle of the 1st arm segment.
        currentShoulderAngle = (armShoulder.getCurrentPosition()/SHOULDER_TICKS_PER_DEGREE)*SHOULDER_GEAR_RATIO;

        // Gets the absolute positioning of the 1st arm segment, assuming it always starts from
        // the "down" position beside the phone mount at START_SHOULDER_ANGLE.
        double adjustedShoulderAngle = currentShoulderAngle + START_SHOULDER_ANGLE;
        telemetry.addData("Shoulder Angle: ", currentShoulderAngle );

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
        telemetry.addData("Elbow Posisiton: ", armElbow.getCurrentPosition());

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

    /*
     * method for children autonomous opmodes to override and insert case specific moves.
     */
    public abstract void main();

}
