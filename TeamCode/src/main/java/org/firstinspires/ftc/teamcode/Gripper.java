package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Rack and Pinion gripper for controlling stones.
 */
public class Gripper extends Motor{

    //motor final min and max
    private int CLOSED_POS = 1100;
    private int OPEN_POS = -2900;

    //the touch sensor on the gripper
    private TouchSensor touchy;

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
        touchy = hardwareMap.get(TouchSensor.class, "touch");
    }

    public void setClosedPosition(int pos){
        CLOSED_POS = pos;
    }

    public void setOpenPosition(int pos){
        OPEN_POS = pos;
    }

    public void setEndpoints(int openPosition, int closedPosition){
        CLOSED_POS = closedPosition;
        OPEN_POS = openPosition;
    }

    //method to check if the motor motor has exceeded its boundaries
    public boolean boundariesExceeded(){
        if (motor.getCurrentPosition() > OPEN_POS && motor.getCurrentPosition() < CLOSED_POS){
            return false;
        }
        return true;
    }

    // Add other motor specific methods i.e.
    public void grabStone(){
        ;
    }

    //method that is ran for the motor during TeleOp
    public void teleopRun(Gamepad gamepad2){
        //programming the motor
        if (gamepad2.right_trigger - gamepad2.left_trigger > 0){
            if (!touchy.isPressed() && !boundariesExceeded()){
                motor.setPower((gamepad2.right_trigger - gamepad2.left_trigger));
            }
            else{
                motor.setPower(0);
            }
        }
        else if (!boundariesExceeded()) {
            motor.setPower((gamepad2.right_trigger - gamepad2.left_trigger));
        }
        else if (boundariesExceeded()){
            if (motor.getCurrentPosition() <= OPEN_POS){
                if (gamepad2.right_trigger - gamepad2.left_trigger > 0) {
                    motor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                }
                else{
                    motor.setPower(0);
                }
            }
            else if (motor.getCurrentPosition() >= CLOSED_POS){
                if (gamepad2.right_trigger - gamepad2.left_trigger < 0){
                    motor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                }
                else{
                    motor.setPower(0);
                }
            }
            else{
                motor.setPower(0);
            }
        }
    }

    public boolean isPressed() {
        return touchy.isPressed();
    }
}
