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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Parent TeleOP class to hold driver controls.
 */


@TeleOp(name="Motor Viewer", group ="Linear Opmode")
//@Disabled
public class MotorViewer extends LinearOpMode {

    //DRIVE TRAIN MOTOR VARIABLES
    // Declare the motor variables
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //map the motors and set default running values
        Motor armShoulder = new Motor(hardwareMap, "arm_shoulder",
                -12, 1440, 1.0/10.0, 0,
                DcMotorSimple.Direction.REVERSE);


        Motor armElbow = new Motor(hardwareMap, "arm_elbow",
                180, 1120, 3.0/8.0, 0.0005,
                DcMotorSimple.Direction.FORWARD);
        /* 164
        Motor gripper = new Gripper(hardwareMap, "gripperMotor",
                0, 1440, 3.5, 0,
                DcMotor.Direction.FORWARD);

        */

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        int targetPosition = 90;
        while(opModeIsActive()){



            if (gamepad2.a){
                armShoulder.to(90);
            }
            else {
                // Set power to the input cubed to give more control at the center of the control range
                armShoulder.setPower(Math.pow(gamepad2.right_stick_y, 3));
                armElbow.setPower(Math.pow(gamepad2.left_stick_y, 3)/3);
            }

            telemetry.addData("Shoulder Position", armShoulder.motor.getCurrentPosition());
            telemetry.addData("Shoulder Angle", armShoulder.getCurrentAngle());
            telemetry.addData("Elbow Position", armElbow.motor.getCurrentPosition());
            telemetry.addData("Elbow Angle", armElbow.getCurrentAngle());

            telemetry.update();




        }


    }
}
