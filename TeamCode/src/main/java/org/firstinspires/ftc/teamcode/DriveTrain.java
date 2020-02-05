package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

    // Set Up four drive motors, though this class could be implemented with only two.
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Drivetrain can be controlled by either power or encoder, this keeps track of which mode is current
    private boolean isPowerMode = true;

    // Targets for running by encoder
    private int frontRightTarget;
    private int frontLeftTarget;
    private int backRightTarget;
    private int backLeftTarget;

    // Initial positions for running by encoder
    private int frontRightStart;
    private int frontLeftStart;
    private int backRightStart;
    private int backLeftStart;

    //Default drive speeds
    private double drivePower = 0.75;
    private double slowPower = 0.45;

    //Declare encoder variables
    static final double COUNTS_PER_MOTOR_TETRIX = 1440;    // Tetrix Matrix 12V motor with 52.8:1 gearbox
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 2.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public DriveTrain() {

    }

    public DriveTrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        reverseRight();
        setBrakeMode();
    }

    public DriveTrain(HardwareMap hardwareMap, String frontLeft, String frontRight, String backLeft, String backRight){
        this.frontLeft = hardwareMap.get(DcMotor.class, frontLeft);
        this.frontRight = hardwareMap.get(DcMotor.class, frontRight);
        this.backLeft = hardwareMap.get(DcMotor.class, backLeft);
        this.backRight = hardwareMap.get(DcMotor.class, backRight);

        reverseRight();
        setBrakeMode();
    }

    public void reverseRight() {
        //reverse the right motors
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void reverseLeft() {
        //reverse the right motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void setBrakeMode(){
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void forward() {
        frontLeft.setPower(drivePower);
        backLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backRight.setPower(drivePower);
    }

    public void backward() {
        frontLeft.setPower(-drivePower);
        backLeft.setPower(-drivePower);
        frontRight.setPower(-drivePower);
        backRight.setPower(-drivePower);
    }

    public void left() {
        frontLeft.setPower(drivePower);
        backLeft.setPower(drivePower);
        frontRight.setPower(-drivePower);
        backRight.setPower(-drivePower);
    }

    public void right() {
        frontLeft.setPower(-drivePower);
        backLeft.setPower(-drivePower);
        frontRight.setPower(drivePower);
        backRight.setPower(drivePower);
    }

    public void stopRobot() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    //accessor methods for power
    public double getDrivePower(){ return drivePower; }

    public double getSlowPower(){ return slowPower; }


    public void powerMode(){
        if (!isPowerMode) {
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isPowerMode = true;
        }
    }
    public void encoderMode(){
        if (isPowerMode){
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);

            frontRightStart = frontRight.getCurrentPosition();
            frontLeftStart = frontLeft.getCurrentPosition();
            backRightStart = backRight.getCurrentPosition();
            backLeftStart = backLeft.getCurrentPosition();

            isPowerMode = false;
        }
    }

    public void encoderDrive(double leftInches, double rightInches, double power){
        // Determine new target position, and pass to motor controller
        frontRightTarget = frontRightStart + (int) (rightInches * COUNTS_PER_INCH);
        frontLeftTarget = frontLeftStart + (int) (leftInches * COUNTS_PER_INCH);
        backRightTarget = backRightStart + (int) (rightInches * COUNTS_PER_INCH);
        backLeftTarget = backLeftStart + (int) (leftInches * COUNTS_PER_INCH);

        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);

        // Turn On RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));
        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));
    }

    public void encoderDrive(double inches, boolean slowMode){
        // Determine new target position, and pass to motor controller
        frontRightTarget = frontRightStart + (int) (inches * COUNTS_PER_INCH);
        frontLeftTarget = frontLeftStart + (int) (inches * COUNTS_PER_INCH);
        backRightTarget = backRightStart + (int) (inches * COUNTS_PER_INCH);
        backLeftTarget = backLeftStart + (int) (inches * COUNTS_PER_INCH);

        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);

        // Turn On RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double power = 0;
        // start motion.
        if (slowMode){
            power = drivePower;
        }
        else{
            power = slowPower;
        }

        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));
        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));

    }

    public void encoderTurn(double degrees, double inchesPerDegrees, boolean slowMode) {
        double power = getDrivePower();
        if (slowMode){
            power = getSlowPower();
        }
        encoderDrive(-degrees * inchesPerDegrees * 2, degrees * inchesPerDegrees * 2, power);
    }


    public boolean isBusy(){
        return (backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontLeft.isBusy());
        // Or for a safer option, quit when any motor is finished;
        //return (backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontLeft.isBusy());
    }
}
