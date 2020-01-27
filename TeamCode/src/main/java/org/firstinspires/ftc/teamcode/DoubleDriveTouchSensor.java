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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="Double Drive Touch Sensor", group="Linear Opmode")

public class DoubleDriveTouchSensor extends TeleOP {

    @Override
    public void main() {

        waitForStart();
        TouchSensor touchy;

        while(opModeIsActive()) {

            // Set wheel power to cube of stick value to give more control near the center:
            f_leftDrive.setPower(-Math.pow(gamepad1.right_stick_y, 3));
            b_leftDrive.setPower(-Math.pow(gamepad1.right_stick_y, 3));
            f_rightDrive.setPower(-Math.pow(gamepad1.left_stick_y, 3));
            b_rightDrive.setPower(-Math.pow(gamepad1.left_stick_y, 3));
            touchy = hardwareMap.get(TouchSensor.class, "touch");

            //SETUP the claw to work with the controller
            if (gamepad1.y)
            {
                clawUp(.25);
            }

            else if (gamepad1.a)
            {
                clawDown(1);
            }


            // Run gripper according to triggers.
            if (touchy.isPressed()) {
                if (gamepad2.right_trigger-gamepad2.left_trigger < 0) {
                    gripperMotor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                    telemetry.addData("Status: ", "Touching Block");
                }
            }
            else{
                gripperMotor.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
                telemetry.addData("Status: ", "NOT Touching Block");
            }

            // Reset the arm encoders if the arm gets out of sync from gear slippage.
            if (gamepad2.x){
                armReset();
            }

            /*if (gamepad2.a){
                armTo(.4, 190, -100, 2);
            }


            if (gamepad2.b) {
                armTo(.4, START_SHOULDER_ANGLE,-100, 3);
            }

            if (gamepad2.y) {
                armTo(.4, START_SHOULDER_ANGLE, START_ELBOW_ANGLE, 2);
            }
            */

            /*
            if (armElbow.getMode()!= DcMotor.RunMode.RUN_TO_POSITION &&
                    armElbow.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {// Set shoulder power to the right stick, adjusted for position aid compensation
                double sPower = gamepad2.right_stick_y + calculateShoulderAid();
                armShoulder.setPower(sPower);
                telemetry.addData("shoulder power", sPower);

                // Set elbow power to the left stick, adjusted for position aid compensation
                double ePower = gamepad2.left_stick_y / 3 + calculateElbowAid();
                armElbow.setPower(ePower);
                telemetry.addData("elbow power", ePower);
            }

             */
            /*if (gamepad2.left_bumper || gamepad2.right_bumper) {
                stopContinuousArm();
            }
            */


            double sPower = gamepad2.right_stick_y + calculateShoulderAid();
            armShoulder.setPower(sPower);
            telemetry.addData("shoulder power", sPower);

            // Set elbow power to the left stick, adjusted for position aid compensation
            double ePower = gamepad2.left_stick_y / 3 + calculateElbowAid();
            armElbow.setPower(ePower);
            telemetry.addData("elbow power", ePower);


            telemetry.update();
        }
    }

    // reset the arm function during play to account for slippage.
    public void armReset(){
        armShoulder.setPower(0);
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armElbow.setPower(0);
        armElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // SEt initial angle to the angle the 1st arm segment is at when resting on the robot (degrees) ;
        currentShoulderAngle = START_SHOULDER_ANGLE;

        // SEt initial angle to the angle the 2nd arm segment is at when raesting on the robot (degrees) ;
        currentElbowAngle = START_ELBOW_ANGLE;
    }

}