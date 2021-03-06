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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder Drive", group="Linear Opmode")
public class EncoderTest extends LinearOpMode {
    //DRIVE TRAIN MOTOR VARIABLES
    // Declare the motor variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor f_leftDrive = null;
    private DcMotor f_rightDrive = null;
    private DcMotor b_leftDrive = null;
    private DcMotor b_rightDrive = null;

    //static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // REV HD Hex motor with 40:1 gearbox
    static final double     COUNTS_PER_MOTOR_TETRIX   = 1478.4;    // Tetrix Matrix 12V motor with 52.8:1 gearbox
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        //DRIVE TRAIN MOTOR SETUP
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        f_leftDrive  = hardwareMap.get(DcMotor.class, "f_leftDrive");
        f_rightDrive = hardwareMap.get(DcMotor.class, "f_rightDrive");
        b_leftDrive = hardwareMap.get(DcMotor.class, "b_leftDrive");
        b_rightDrive = hardwareMap.get(DcMotor.class, "b_rightDrive");

        //reverse the right motors
        f_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        b_rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //set the motors to brake when power equals zero
        f_rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b_rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        f_leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b_leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setup the values that are needed
        double driveSensitivity = 1.5;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // we have determined that speed is the indicator for direction
        //should we add a direction parameter to determine direction?
        // test case: try to move one motor zero inches and the other in a positive inches direction
        encoderDrive(1.0,10,10,30);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


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
        leftInches /= 2.4;
        rightInches /= 2.4;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newf_RightTarget = f_rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newf_LeftTarget = f_leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            f_leftDrive.setTargetPosition(newf_LeftTarget);
            f_rightDrive.setTargetPosition(newf_RightTarget);

            // Turn On RUN_TO_POSITION
            //f_leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //f_rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            f_leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            f_rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            // power is set to negative speed to make the bot move in the forward direction established by the builders
            runtime.reset();
            f_leftDrive.setPower(speedD);
            f_rightDrive.setPower(speedD);

            // set the front wheels to move with the back wheels
            b_leftDrive.setPower(speedD);
            b_rightDrive.setPower(speedD);

            // keep looping while we are still active, and there is time left, and all motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when ANY motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that ALL motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (f_leftDrive.isBusy() && f_rightDrive.isBusy())) {

                // Display data for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newf_LeftTarget, newf_RightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
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
    }

}
