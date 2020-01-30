package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    //Declare the variables for the claw servos
    Servo leftClaw = null;
    Servo rightClaw = null;

    private final double clawUpPosition = 1.0;
    private final double clawDownPosition = 0.25;

    public Claw() {
        // Do nothing
        // Use other Constructor
    }

    public Claw(HardwareMap hardwareMap, String leftServoDeviceName, String righttServoDeviceName){
        //map the claw servos
        leftClaw = hardwareMap.get(Servo.class, leftServoDeviceName);
        rightClaw = hardwareMap.get(Servo.class, righttServoDeviceName);

        //reverse one of the claw servos
        rightClaw.setDirection(Servo.Direction.REVERSE);
    }

    public Claw(Servo left, Servo right){
        leftClaw = left;
        rightClaw = right;

        //reverse one of the claw servos
        rightClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void down(){
        leftClaw.setPosition(clawDownPosition);
        rightClaw.setPosition(clawUpPosition);
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
