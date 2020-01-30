package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper extends Motor{

    //gripper final min and max
    private int GRIPPER_CLOSED_POS = 1100;
    private int GRIPPER_OPEN_POS = -2900;

    public Gripper() {
        // Does nothing
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
    public Gripper(HardwareMap hardwareMap, String deviceName, double start_angle,
                 double encoder_ticks_per_rotation, double gear_ratio, double max_aid,
                 DcMotorSimple.Direction direction){
        // Super refers to the parent class, in this case, Motor.
        super(hardwareMap, deviceName);

        setStartAngle(start_angle);
        setTicksPerRotation(encoder_ticks_per_rotation);
        setGearRatio(gear_ratio);
        setMaxAid(max_aid);
        setDirection(direction);


    }

    public void setClosedPosition(int pos){
        GRIPPER_CLOSED_POS = pos;
    }

    public void setOpenPosition(int pos){
        GRIPPER_OPEN_POS = pos;
    }

    public void setEndpoints(int openPosition, int closedPosition){
        GRIPPER_CLOSED_POS = closedPosition;
        GRIPPER_OPEN_POS = openPosition;
    }

    //method to check if the gripper motor has exceeded its boundaries
    public boolean boundariesExceeded(){
        if (motor.getCurrentPosition() > GRIPPER_OPEN_POS && motor.getCurrentPosition() < GRIPPER_CLOSED_POS){
            return false;
        }
        return true;
    }

    // Add other gripper specific methods i.e.
    public void grabStone(){
        ;
    }
}
