package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gripper extends Motor{

    //motor final min and max
    //private int CLOSED_POS = 1100;
    //private int OPEN_POS = -2900;

    private int CLOSED_POS = 950;//1200
    private int OPEN_POS = -1900;

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
        super(hardwareMap, deviceName, 0, encoder_ticks_per_rotation, gear_ratio, max_aid, direction);

        touchy = hardwareMap.get(TouchSensor.class, "touch");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Gripper(HardwareMap hardwareMap, String deviceName, double start_angle,
                   double encoder_ticks_per_rotation, double gear_ratio, double max_aid,
                   DcMotorSimple.Direction direction, AutoOP_ClassBased op){
        // Super refers to the parent class, in this case, Motor.
        super(hardwareMap, deviceName, 0, encoder_ticks_per_rotation, gear_ratio, max_aid, direction, op);

        touchy = hardwareMap.get(TouchSensor.class, "touch");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setClosedPosition(int pos){
        CLOSED_POS = pos;
    }

    public void setOpenPosition(int pos){
        OPEN_POS = pos;
    }

    public double getOpenPosition() { return OPEN_POS; }

    public double getClosedPosition() { return CLOSED_POS; }

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

    //method to check if the max boundary is exceeded
    public boolean maxBoundaryExceeded() { return motor.getCurrentPosition() <= OPEN_POS; }

    //method to check if the min boundary is exceeded
    public boolean minBoundaryExceeded() { return motor.getCurrentPosition() >= CLOSED_POS; }

    //method to return true or false depending on whether the touch sensor is pressed
    public boolean isTouching() { return touchy.isPressed(); }

    //method

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

    public void run(Gamepad gp, Telemetry telemetry){
        //programming the motor
        double power = gp.right_trigger - gp.left_trigger;
        if (maxBoundaryExceeded() && power < 0){
            power = 0;
        }
        if (minBoundaryExceeded() && power > 0 || isTouching() && power > 0) {
            power = 0;
        }
        motor.setPower(power);
    }

    //method to run to a position
    public void toPosition(int target){
        motor.setTargetPosition(target);

        // Set drive power
        motor.setPower(Math.abs(getPower()));

        // Turn On RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //method to close until the touch sensor is pressed
    //temp closing to boundary position
    public void closeUntilTouching(){
        toPosition(CLOSED_POS);
        while (opmode.opModeIsActive() && !isTouching()){
            ;
        }
        motor.setPower(0);

    }

    public void closeGripper(double timeout){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        if (opmode.opModeIsActive()){
            toPosition(CLOSED_POS);
            while (opmode.opModeIsActive() && time.seconds() < timeout && !isTouching() && motor.isBusy()){
                ;
            }

            //stop all motion (yeah)
            motor.setPower(0);
        }

    }

    //overloaded version of close gripper with default timeout value that we acquired from TESTING
    public void closeGripper() {
        closeGripper(1.2);
    }

    //method to open the gripper (goes to max boundary)
    public void openMax(){
        toPosition(OPEN_POS);
    }

    //method to open the gripper
    public void openGripper(double timeout){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        if (opmode.opModeIsActive()){
            openMax();
            while (opmode.opModeIsActive() && time.seconds() < timeout && motor.isBusy()){
                ;
            }

            //stop all motion (yeah)
            motor.setPower(0);
        }
    }

    //overloaded version of open gripper with default timeout value that we acquired from TESTING
    public void openGripper(){
        openGripper(1.2);
    }

    //method to grab the stone
    public void grabStone(DriveTrain driveTrain){
        openMax();
        driveTrain.encoderDrive(5, false);
        closeUntilTouching();
        driveTrain.encoderDrive(-5, false);
    }


}
