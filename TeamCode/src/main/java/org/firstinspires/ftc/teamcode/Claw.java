package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw{
    //Declare the variables for the claw servos
    Servo leftClaw = null;
    Servo rightClaw = null;

    private final double clawUpPosition = 1.0;
    private final double clawDownPosition = 0.25;

    //auto op variable to use opModeIsActive()
    AutoOP_ClassBased opmode = null;

    public Claw() {
        // Do nothing
        // Use other Constructor
    }

    public Claw(HardwareMap hardwareMap, String leftServoDeviceName, String rightServoDeviceName, AutoOP_ClassBased op){
        //map the claw servos
        leftClaw = hardwareMap.get(Servo.class, leftServoDeviceName);
        rightClaw = hardwareMap.get(Servo.class, rightServoDeviceName);

        //reverse one of the claw servos
        rightClaw.setDirection(Servo.Direction.REVERSE);

        opmode = op;
    }

    public Claw(Servo left, Servo right, AutoOP_ClassBased op){
        leftClaw = left;
        rightClaw = right;

        //reverse one of the claw servos
        rightClaw.setDirection(Servo.Direction.REVERSE);

        opmode = op;
    }

    public void down(){
        leftClaw.setPosition(clawDownPosition);
        rightClaw.setPosition(clawDownPosition);
    }
    public void down(double position){
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }

    public void up(){
        rightClaw.setPosition(clawUpPosition);
        leftClaw.setPosition(clawUpPosition);
    }

    public void up(double position){
        leftClaw.setPosition(position);
        rightClaw.setPosition(position);
    }

}
