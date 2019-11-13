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


@TeleOp(name="Elbow Aid", group="Linear Opmode")
//@Disabled
public class TeleOPElbowArmAid extends TeleOP {

    @Override
    public void main() {
        //Code to send controls to robot
        waitForStart();

        // Reset encoder counts;
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        final double MAX_SHOULDER_AID = 0.5;
        final double MAX_ELBOW_AID = 0.25;



        while(opModeIsActive()){

            // Shoulder joint controls
            telemetry.addData("Shoulder Posisiton: ", armShoulder.getCurrentPosition());

            // Use the position of the encoder and the known starting position of the arm to
            // determine the angle of the 1st arm segment.
            currentShoulderAngle = (armShoulder.getCurrentPosition()/SHOULDER_TICKS_PER_DEGREE)*SHOULDER_GEAR_RATIO;

            // Gets the absolute positioning of the 1st arm segment, assuming it always starts from
            // the "down" position beside the phone mount at START_SHOULDER_ANGLE.
            currentShoulderAngle = currentShoulderAngle + START_SHOULDER_ANGLE;
            telemetry.addData("Shoulder Angle: ", currentShoulderAngle );

            // Uses cosine to determine the appropriate aid to add to the arm to hold it stationary:
            /*        _- 0 - _
                    /         \
                 -max         max
                    \         /
                       - 0 -             */
            double shoulderAid = MAX_SHOULDER_AID * Math.cos(Math.toRadians(currentShoulderAngle));
            telemetry.addData("Shoulder Aid: ", shoulderAid);


            //Elbow joint controls
            telemetry.addData("Elbow Posisiton: ", armElbow.getCurrentPosition());

            // Three things determine the angle of the second arm segment.
            // 1. position of the encoder
            // 2. known starting position of the arm
            // 3. The angle of the origin (angle of the 1st arm segment)
            currentElbowAngle = (armElbow.getCurrentPosition()/ELBOW_TICKS_PER_DEGREE)*ELBOW_GEAR_RATIO;
            double adjustedElbowAngle = -currentElbowAngle + START_ELBOW_ANGLE+currentShoulderAngle;
            telemetry.addData("adjusted Elbow Angle: ", adjustedElbowAngle);
            telemetry.addData("Elbow Angle: ", currentElbowAngle);

            // Uses cosine to determine aid using same logic as first segment
            double elbowAid = MAX_ELBOW_AID * Math.cos(Math.toRadians(currentElbowAngle));
            telemetry.addData("Elbow Aid: ", elbowAid);



            telemetry.update();



        }
    }
}