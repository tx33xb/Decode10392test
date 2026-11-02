package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RobotCentricTeleOp extends LinearOpMode {
    // Highest power any drive motor is given (1.0 = full power).
    private static final double MAX_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl_motor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("back_right_motor");

        // Reverse the left side motors. This robot needs the left side reversed.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * MAX_POWER;
            double backLeftPower = (y - x + rx) / denominator * MAX_POWER;
            double frontRightPower = (y - x - rx) / denominator * MAX_POWER;
            double backRightPower = (y + x - rx) / denominator * MAX_POWER;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
