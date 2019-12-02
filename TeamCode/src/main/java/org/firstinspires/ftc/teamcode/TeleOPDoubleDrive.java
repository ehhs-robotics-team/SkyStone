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


@TeleOp(name="Drive", group="Linear Opmode")

public class TeleOPDoubleDrive extends TeleOP {

    @Override
    public void main() {

        waitForStart();

        while(opModeIsActive()) {

            // Set wheel power to cube of stick value to give more control near the center:
            f_leftDrive.setPower(-Math.pow(gamepad1.right_stick_y, 3));
            b_leftDrive.setPower(-Math.pow(gamepad1.right_stick_y, 3));
            f_rightDrive.setPower(-Math.pow(gamepad1.left_stick_y, 3));
            b_rightDrive.setPower(-Math.pow(gamepad1.left_stick_y, 3));

            //SETUP the claw to work with the controller
            if (gamepad1.y) {
                leftClaw.setPosition(clawUpPosition);
                rightClaw.setPosition(clawUpPosition);
            } else if (gamepad1.a) {
                rightClaw.setPosition(clawDownPosition);
                leftClaw.setPosition(clawDownPosition);
            } else {
                //double restPosition = 0.75;
                //rightClaw.setPosition(restPosition);
                //leftClaw.setPosition(restPosition);
            }


            // Set shoulder power to the right trigger, negative or positive depending on the bumper
            double shoulderAid = calculateShoulderAid();
            double sPower = gamepad1.right_stick_y+shoulderAid;
            armShoulder.setPower(sPower);
            telemetry.addData("shoulder power", sPower);


            // Set elbow power to the left trigger, negative or positive depending on the bumper
            double ePower = gamepad2.left_stick_y/3 + calculateElbowAid();
            armElbow.setPower(ePower);
            telemetry.addData("elbow power", ePower);

            double gripperPower = 0.5;
            if (gamepad2.dpad_right){
                gripperServo.setPower(gripperPower);
            } else if (gamepad2.dpad_left){
                gripperServo.setPower(-gripperPower);
            } else {
                gripperServo.setPower(0);
            }

            telemetry.update();







        }
    }

    // reset the arm function during play to account for slippage.
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

}