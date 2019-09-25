package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BellaTest (Blocks to Java)", group = "")
public class BellaTest extends LinearOpMode {

  private DcMotor right1;
  private DcMotor right2;
  private DcMotor left1;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    float right12;
    float right22;
    float left12;

    right1 = hardwareMap.dcMotor.get("right1");
    right2 = hardwareMap.dcMotor.get("right2");
    left1 = hardwareMap.dcMotor.get("left1");

    // Initialization blocks
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        right12 = gamepad1.left_stick_y;
        right22 = gamepad1.left_stick_y;
        left12 = gamepad1.right_stick_y;
        right1.setPower(right12);
        right2.setPower(right22);
        left1.setPower(left12);
        telemetry.addData("Right_PowerFront", right12);
        telemetry.addData("Right_PowerBack", right12);
        telemetry.addData("Left_Power", right12);
        telemetry.addData("Motor Power", right1.getPower());
        telemetry.addData("Motor Power", right2.getPower());
        telemetry.addData("Motor Power", left1.getPower());
        telemetry.update();
      }
    }
  }
}
