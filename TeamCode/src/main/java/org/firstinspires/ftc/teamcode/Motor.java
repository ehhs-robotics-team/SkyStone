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

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndInstance;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeServices;

/**
 * Motor class that enables both driving by encoder or by power
 */
public class Motor{

    private double START_ANGLE = 0;
    private double TICKS_PER_ROTATION = 1440;
    private double TICKS_PER_DEGREE = TICKS_PER_ROTATION/360;
    private double GEAR_RATIO = 1/1;
    private double MAX_AID = 0;
    private int target;
    private double currentAngle;
    private double defaultPower = 0.5;

    // Motor can be controlled by either power or encoder, this keeps track of which mode is current
    private boolean isPowerMode = true;

    DcMotor motor = null;

    AutoOP_ClassBased opmode = null;

    /**
     * Does nothing
     * Use other constructors
     */
    public Motor() {
        ;
    }

    /**
     * Motor class that enables both driving by encoder or by power
     * @param hardwareMap Robot's Hardware Map
     * @param deviceName
     * @param start_angle
     * @param encoder_ticks_per_rotation
     * @param gear_ratio
     * @param max_aid
     * @param direction
     */
    public Motor(HardwareMap hardwareMap, String deviceName, double start_angle,
                 double encoder_ticks_per_rotation, double gear_ratio, double max_aid,
                 DcMotorSimple.Direction direction, AutoOP_ClassBased op){
        this(hardwareMap, deviceName);

        setStartAngle(start_angle);
        setTicksPerRotation(encoder_ticks_per_rotation);
        setGearRatio(gear_ratio);
        setMaxAid(max_aid);
        setDirection(direction);
        opmode = op;
    }

    public Motor(HardwareMap hardwareMap, String deviceName, double start_angle,
                 double encoder_ticks_per_rotation, double gear_ratio, double max_aid,
                 DcMotorSimple.Direction direction){
        this(hardwareMap, deviceName);

        setStartAngle(start_angle);
        setTicksPerRotation(encoder_ticks_per_rotation);
        setGearRatio(gear_ratio);
        setMaxAid(max_aid);
        setDirection(direction);
    }

    /**
     * Motor class that enables both driving by encoder or by power
     * @param hardwareMap
     * @param deviceName
     */
    public Motor(HardwareMap hardwareMap, String deviceName) {
        //map the motors
        motor = hardwareMap.get(DcMotor.class, deviceName);
        reset();
    }

    /**
     * Motor class that enables both driving by encoder or by power
     * @param dcMotor DcMotor
     */
    public Motor (DcMotor dcMotor) {
        motor = dcMotor;
        reset();
    }

    /**
     * Motor class that enables both driving by encoder or by power
     * @param dcMotor
     * @param start_angle
     * @param encoder_ticks_per_rotation
     * @param gear_ratio
     * @param max_aid
     * @param direction
     */
    public Motor(DcMotor dcMotor, double start_angle,
                 double encoder_ticks_per_rotation, double gear_ratio, double max_aid,
                 DcMotorSimple.Direction direction){
        this(dcMotor);

        setStartAngle(start_angle);
        setTicksPerRotation(encoder_ticks_per_rotation);
        setGearRatio(gear_ratio);
        setMaxAid(max_aid);
        setDirection(direction);
    }


    /*
     * Methods to set the default settings of the motor
     */

    /**
     * @param start_angle
     */
    public void setStartAngle(double start_angle){
        START_ANGLE = start_angle;
    }

    public void setTicksPerRotation(double encoder_ticks_per_rotation){
        TICKS_PER_ROTATION = encoder_ticks_per_rotation;
        TICKS_PER_DEGREE = TICKS_PER_ROTATION/360;
    }

    public void setGearRatio(double gear_ratio){
        GEAR_RATIO = gear_ratio;
    }

    public void setMaxAid(double max_aid){
        MAX_AID = max_aid;
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }

    public void setPower(double p){
        powerMode();
        defaultPower = p;
        motor.setPower(defaultPower);
    }

    public double getPower(){
        return defaultPower;
    }

    public void powerMode(){
      if (!isPowerMode) {
          motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          isPowerMode = true;
      }
    }
    public void encoderMode(){
      if (isPowerMode){
        motor.setPower(0);
        isPowerMode = false;
      }
    }

    public double calculateAid(double dependentAngle, Telemetry telemetry){
        //Elbow joint controls
        //telemetry.addData("Elbow Posisiton: ", armElbow.getCurrentPosition());

        // Three things determine the angle of the second arm segment.
        // 1. position of the encoder
        // 2. known starting position of the arm
        // 3. The angle of the origin (angle of the 1st arm segment)
        double rawAngle = (motor.getCurrentPosition()/TICKS_PER_DEGREE)*GEAR_RATIO;
        currentAngle = -rawAngle + START_ANGLE +dependentAngle;
        telemetry.addData("Elbow Angle: ", currentAngle);

        // Uses cosine to determine aid using same logic as first segment
        double aid = MAX_AID * Math.cos(Math.toRadians(currentAngle));
        telemetry.addData("Elbow Aid: ", aid);


        return aid;
    }

    public double calculateAid(Telemetry telemetry){
        return calculateAid(0, telemetry);
    }

    public int calculateTarget(double degrees, double dependentAngle, Telemetry telemetry) {
        // Determine new target position, and display to telemetry
        telemetry.addData("CurrentAngle", getCurrentAngle());
        telemetry.addData("CurrentPos", motor.getCurrentPosition());
        target = calculateTarget(degrees, dependentAngle);
        telemetry.addData("GoalAngle", degrees);
        telemetry.addData("GoalPos", target);
        return target;
    }

    public int calculateTarget(double degrees, double dependentAngle){
        // Determine new target position
        target = (int) (((degrees-START_ANGLE)-dependentAngle) * TICKS_PER_DEGREE / GEAR_RATIO);
        return target;
    }

    public void toTarget(int newTarget, double power){
        target = newTarget;
        encoderMode();
        // Determine new target position, and pass to motor controller
        motor.setTargetPosition(target);

        // Set drive power
        motor.setPower(Math.abs(power));

        // Turn On RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void to(int degrees, Telemetry telemetry){
        to(degrees);
        telemetry.addData("Position1", motor.getCurrentPosition());
        telemetry.addData("Angle1", getCurrentAngle());

    }

    public void to(int degrees, double power, double dependentAngle){
        target = calculateTarget(degrees,dependentAngle);
        toTarget(target, 0.4);
    }
    public void to(int degrees, double power){
        to(degrees, power, 0);
    }

    public void to (int degrees){
        to(degrees, defaultPower);
    }

    // reset the arm function during play to account for slippage.
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set initial angle to the resting position the robot (degrees) ;
        currentAngle = START_ANGLE;
    }

    //method to get current angle
    public double getCurrentAngle(){
        currentAngle = (motor.getCurrentPosition()/TICKS_PER_DEGREE)*GEAR_RATIO;
        return currentAngle + START_ANGLE;
    }
}
