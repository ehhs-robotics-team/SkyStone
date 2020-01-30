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


@TeleOp(name="Dual Mode Arm Control", group="Linear Opmode")

public class ArmControl extends TeleOP {

    public int newShoulderTarget;
    public int newElbowTarget;
    @Override
    public void main() {

        waitForStart();
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean isPowerMode = false;

        while (opModeIsActive()) {

            if (gamepad2.dpad_down) {
                if (isPowerMode) {
                    armShoulder.setPower(0);
                    isPowerMode = false;
                }

                if(gamepad2.a){
                    shoulderTo(90);
                }


            } else {
                // Run using power input from drive controls.
                if (!isPowerMode) {
                    armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    isPowerMode = true;
                }

                // Set power to the input cubed to give more control at the center of the control range
                armShoulder.setPower(Math.pow(gamepad2.right_stick_y, 3));
            }
        }
    }

    public void shoulderTo(int degrees, double power){
        // Determine new target position, and pass to motor controller
        newShoulderTarget = (int) ((degrees + START_SHOULDER_ANGLE) * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
        armShoulder.setTargetPosition(newShoulderTarget);

        // Set drive power
        armShoulder.setPower(Math.abs(power));

        // Display it for the driver.
        telemetry.addData("Shoulder", "Running at %7d to %7d",
                armShoulder.getCurrentPosition(), newElbowTarget);

        // Turn On RUN_TO_POSITION
        armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void shoulderTo(int degrees){
        shoulderTo(degrees, 0.4);
    }
}