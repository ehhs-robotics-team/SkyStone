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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Encoder Arm test", group="Linear Opmode")
public class EncoderArm extends TeleOP {

    @Override
    public void main() {
        encoderTEST1();
        /*armShoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        encoderArm(.1, 20.0, 3);

         */

    }
    public void encoderTEST1(){
        //Code to send controls to robot
        waitForStart();
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newArmTarget = 0;
        armShoulder.setTargetPosition(newArmTarget);
        armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armShoulder.setPower(0.1);

        while(opModeIsActive()) {
            //  //right wheels
            //  f_rightDrive.setPower(gamepad1.right_stick_y / driveSensitivity);
            //  b_rightDrive.setPower(gamepad1.right_stick_y / driveSensitivity);

            if (gamepad1.x) {
                newArmTarget += 1;
            } else if (gamepad1.b) {
                newArmTarget -= 1;
            }
            telemetry.addData("Target: ", newArmTarget);
            telemetry.addData("Current: ", armShoulder.getCurrentPosition());
            if (Math.abs(armShoulder.getCurrentPosition() - newArmTarget) < 50) {
                //if the current position is within 50 encoder ticks of the target, the motor shuts off
                armShoulder.setPower(0);
                armShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else {
                //if the current position isn't close enough to the target, the arm uses encoders to move
                armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armShoulder.setPower(.1);

                armShoulder.setTargetPosition(newArmTarget);
            }
            //update driver station
            telemetry.update();
        }
    }
    public void encoderArm(double speed,
                             double degrees,
                             double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = armShoulder.getCurrentPosition() + (int) (degrees * 1120.0/360.0);
            armShoulder.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            armShoulder.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
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
}