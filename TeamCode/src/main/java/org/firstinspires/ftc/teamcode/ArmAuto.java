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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Auto Encoder Arm Test", group="Linear Opmode")
//@Disabled
public class ArmAuto extends AutoOP {

    CRServo gripperServo = null;

    DcMotor armShoulder = null;
    DcMotor armElbow = null;


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
    double MAX_SHOULDER_AID = 0.002;
    double MAX_ELBOW_AID = 0.0005;

    @Override
    public void main(){

        //map the gripper's servo
        gripperServo = hardwareMap.get(CRServo.class, "gripperServo");

        armElbow = hardwareMap.get(DcMotor.class, "arm_elbow");
        armShoulder = hardwareMap.get(DcMotor.class, "arm_shoulder");

        armShoulder.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulderReset();


        waitForStart();
        encoderElbow(0.1, -20,5);
        encoderShoulder(0.2, 65, 10);

        //encoderShoulder(.2, 20, 5);


        sleep(3000);

    }

    // reset the arm function during play to account for slippage.
    public void shoulderReset(){
        armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // SEt initial angle to the angle the 1st arm segment is at when resting on the robot (degrees) ;
        currentShoulderAngle = START_SHOULDER_ANGLE;

        // SEt initial angle to the angle the 2nd arm segment is at when raesting on the robot (degrees) ;
        currentElbowAngle = START_ELBOW_ANGLE;
    }


    /* Method to calculate the power added to the shoulder to keep it stationary during play.
     * Employs cosine to keep max power at horizontal position and zero power at vertical.
     */
    public double calculateShoulderAid(){
        // Shoulder joint controls
        //telemetry.addData("Shoulder Posisiton: ", armShoulder.getCurrentPosition());

        // Use the position of the encoder and the known starting position of the arm to
        // determine the angle of the 1st arm segment.
        currentShoulderAngle = (armShoulder.getCurrentPosition()/SHOULDER_TICKS_PER_DEGREE)*SHOULDER_GEAR_RATIO;

        // Gets the absolute positioning of the 1st arm segment, assuming it always starts from
        // the "down" position beside the phone mount at START_SHOULDER_ANGLE.
        double adjustedShoulderAngle = currentShoulderAngle + START_SHOULDER_ANGLE;
        telemetry.addData("Shoulder Angle: ", adjustedShoulderAngle );

        // Uses cosine to determine the appropriate aid to add to the arm to hold it stationary:
            /*        _- 0 - _
                    /         \
                 -max         max
                    \         /
                       - 0 -             */
        double shoulderAid = MAX_SHOULDER_AID * Math.cos(Math.toRadians(adjustedShoulderAngle));
        telemetry.addData("Shoulder Aid: ", shoulderAid);

        return shoulderAid;
    }

    /* Method to calculate the power added to the elbow to keep it stationary during play.
     * Automatically adjusts aid for the varying position of the shoulder.
     * Employs cosine to keep max power at horizontal position and zero power at vertical.
     */


    public double calculateElbowAid(){
        //Elbow joint controls
        //telemetry.addData("Elbow Posisiton: ", armElbow.getCurrentPosition());

        // Three things determine the angle of the second arm segment.
        // 1. position of the encoder
        // 2. known starting position of the arm
        // 3. The angle of the origin (angle of the 1st arm segment)
        currentElbowAngle = (armElbow.getCurrentPosition()/ELBOW_TICKS_PER_DEGREE)*ELBOW_GEAR_RATIO;
        double adjustedElbowAngle = -currentElbowAngle + START_ELBOW_ANGLE +currentShoulderAngle;
        telemetry.addData("Elbow Angle: ", adjustedElbowAngle);

        // Uses cosine to determine aid using same logic as first segment
        double elbowAid = MAX_ELBOW_AID * Math.cos(Math.toRadians(adjustedElbowAngle));
        telemetry.addData("Elbow Aid: ", elbowAid);


        return elbowAid;
    }

    /*  Method to perform a relative move of the shoulder section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderShoulder(double speed,
                           double degrees,
                           double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = armShoulder.getCurrentPosition() + (int) (degrees * SHOULDER_TICKS_PER_DEGREE / SHOULDER_GEAR_RATIO);
            newTarget *= 2;
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
            armShoulder.setPower(calculateShoulderAid());

            // Turn off RUN_TO_POSITION
            armShoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    /*  Method to perform a relative move of the elbow section of the arm, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderElbow(double speed, double degrees, double timeoutS){
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = armElbow.getCurrentPosition() + (int) (-degrees * ELBOW_TICKS_PER_DEGREE/ELBOW_GEAR_RATIO);
            newTarget *= 2;
            armElbow.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            armElbow.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (armElbow.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Elbow", "Running at %7d to %7d",
                        armElbow.getCurrentPosition(), newTarget);
                telemetry.update();
            }

            // Stop all motion;
            armElbow.setPower(calculateElbowAid());

            // Turn off RUN_TO_POSITION
            armElbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }

    }

    public void openGripper(double seconds){
        gripperServo.setPower(0.5);
        sleep((long)(seconds*1000));
        gripperServo.setPower(0);
    }
    public void closeGripper(double seconds){
        gripperServo.setPower(-0.5);
        sleep((long)(1000*seconds));
        gripperServo.setPower(0);
    }
}