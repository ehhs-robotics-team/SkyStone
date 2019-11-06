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


@TeleOp(name="Shoulder Aid", group="Linear Opmode")
//@Disabled
public class TeleOPArmAid extends TeleOP {

    @Override
    public void main() {
        //Code to send controls to robot
        waitForStart();

        // Reset encoder counts;
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double startShoulderAngle = -12; // SEt initial angle to the angle the arm is at resting on the robot (degrees) ;
        double currentShoulderAngle;


        final double TICKS_PER_ROTATION = 1440;
        final double TICKS_PER_DEGREE = TICKS_PER_ROTATION/360;

        // Aid at the extremities, to keep the arm still at full horizontal extension.
        final double MAX_AID = 0.5;



        while(opModeIsActive()){

            /*
            // Set shoulder power to the right trigger, negative or positive depending on the bumper
            if (gamepad1.right_bumper) {
                armShoulder.setPower(gamepad1.right_trigger);
            } else {
                armShoulder.setPower(-gamepad1.right_trigger);
            }

             */
            telemetry.addData("Posisiton", armShoulder.getCurrentPosition());

            // Use the position of the encoder and the known starting position of the arm to determine the angle of the 1st arm segment.
            currentShoulderAngle = armShoulder.getCurrentPosition()/TICKS_PER_DEGREE + startShoulderAngle;

            // Cosine to determine the appropriate aid to add to the arm to hold it stationary:
            /*
                      _- 0 - _
                    /         \
                 -max         max
                    \         /
                       - 0 -
             */
            double aid = MAX_AID * Math.cos(Math.toRadians(currentShoulderAngle));
            telemetry.addData("Shoulder Aid", aid);
        }
    }
}