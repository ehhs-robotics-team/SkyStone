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
import org.firstinspires.ftc.teamcode.TeleOP;


@TeleOp(name="TeleOP Drive Test", group="Linear Opmode")

public class TeleOPTest extends TeleOP {

    @Override
    public void main() {

        while(opModeIsActive()){
            //MAP DRIVE TRAIN TO CONTROLLER
            // Send calculated power to wheels
            //left wheels
            f_leftDrive.setPower(gamepad1.left_stick_y / driveSensitivity);
            b_leftDrive.setPower(gamepad1.left_stick_y / driveSensitivity);


            //right wheels
            f_rightDrive.setPower(gamepad1.right_stick_y / driveSensitivity);
            b_rightDrive.setPower(gamepad1.right_stick_y / driveSensitivity);

            //SETUP the claw to work with the controller
            if (gamepad1.y) {
                leftClaw.setPosition(clawUpPosition);
                rightClaw.setPosition(clawUpPosition);
            }
            if (gamepad1.b) {
                rightClaw.setPosition(clawDownPosition);
                leftClaw.setPosition(clawDownPosition);
            }


            //SETUP and add the ability to use the gripper; bind controls on the controller to operate the gripper's servo
            if (gamepad1.left_bumper) {
                gripperServo.setPower(1);
            } else if (gamepad1.right_bumper) {
                gripperServo.setPower(-1);
            } else {
                gripperServo.setPower(0);
            }
        }
    }
}