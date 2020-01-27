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


@TeleOp(name="Move Arm Test", group="Linear Opmode")

public class ArmMoveTest extends TeleOP {

    public int newShoulderTarget;
    public int newElbowTarget;
    @Override
    public void main() {

        waitForStart();



        while(opModeIsActive()) {
            currentElbowAngle = (armElbow.getCurrentPosition()/ELBOW_TICKS_PER_DEGREE)*ELBOW_GEAR_RATIO + START_ELBOW_ANGLE;
            currentShoulderAngle = (armShoulder.getCurrentPosition()/SHOULDER_TICKS_PER_DEGREE)*SHOULDER_GEAR_RATIO+START_SHOULDER_ANGLE;
            telemetry.addData("Shoulder Position", armShoulder.getCurrentPosition());
            telemetry.addData("Shoulder Angle", currentShoulderAngle);
            telemetry.addData("Elbow Position", armElbow.getCurrentPosition());
            telemetry.addData("Elbow Angle", currentElbowAngle);


            if (gamepad2.left_bumper){
                moveArm(1, 0);
            }

            if (gamepad2.right_bumper) {

                moveArm(-1,0);
            }
        }
    }

    public void moveArm(double shoulderSpeed, double elbowSpeed){

        // Determine new target position, and pass to motor controller
        newElbowTarget += elbowSpeed*20; // elbowDegrees* ELBOW_TICKS_PER_DEGREE/ELBOW_GEAR_RATIO;
        armElbow.setTargetPosition(newElbowTarget);

        // Determine new target position, and pass to motor controller
        newShoulderTarget += shoulderSpeed*20; // shoulderDegrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO;
        armShoulder.setTargetPosition(newShoulderTarget);

        double power = 0.6;
        armElbow.setPower(Math.abs(power*elbowSpeed));
        armShoulder.setPower(Math.abs(power*shoulderSpeed));

        // Display it for the driver.
        telemetry.addData("Elbow", "Running at %7d to %7d",
                armElbow.getCurrentPosition(), newElbowTarget);
        telemetry.addData("Shoulder", "Running at %7d to %7d",
                armShoulder.getCurrentPosition(), newElbowTarget);
        telemetry.update();

        // Turn On RUN_TO_POSITION
        armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /*  Method to perform an absolute move of the elbow section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void armTo(double speed, double shoulderDegrees, double elbowDegrees, double timeoutS){
        int newShoulderTarget;
        int newElbowTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newElbowTarget = (int) (-elbowDegrees * ELBOW_TICKS_PER_DEGREE/ELBOW_GEAR_RATIO);
            newElbowTarget *= 2;
            armElbow.setTargetPosition(newElbowTarget);

            // Determine new target position, and pass to motor controller
            newShoulderTarget = + (int) (shoulderDegrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
            // newTarget *= 2;
            armShoulder.setTargetPosition(newShoulderTarget);

            // Turn On RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            encoderTime.reset();
            armElbow.setPower(Math.abs(speed));
            armShoulder.setPower(Math.abs(speed*2));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (encoderTime.seconds() < timeoutS) &&
                    (armElbow.isBusy() || armShoulder.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Elbow", "Running at %7d to %7d",
                        armElbow.getCurrentPosition(), newElbowTarget);
                telemetry.addData("Shoulder", "Running at %7d to %7d",
                        armShoulder.getCurrentPosition(), newElbowTarget);
                telemetry.update();
            }

            // Stop all motion;
            armElbow.setPower(calculateElbowAid());
            armShoulder.setPower(0);

            // Turn off RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }
}