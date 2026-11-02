package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Plain gm0 field-centric mecanum drive, kept as a bare baseline to compare against {@link DriveTeleOp}.
 * No heading hold, ramp, trims, snapping, deadzone or precision mode. Only changed to fit this robot:
 * motor names, motor directions, hub orientation, power cap, and right stick click to reset forward.
 */
@TeleOp(name = "Field Centric Basic", group = "Competition")
public class FieldCentricMecanumTeleOp extends LinearOpMode {

    // Same cap as DriveTeleOp so the comparison is fair (1.0 = full power).
    private static final double MAX_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor  = hardwareMap.dcMotor.get("fl_motor");
        DcMotor backLeftMotor   = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor backRightMotor  = hardwareMap.dcMotor.get("back_right_motor");

        // This robot needs the left side reversed.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hub stands vertically: logo forward, USB left.
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        imu.resetYaw();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Reset field-centric forward to the direction the robot is facing.
            if (gamepad1.right_stick_button) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower  = (rotY + rotX + rx) / denominator * MAX_POWER;
            double backLeftPower   = (rotY - rotX + rx) / denominator * MAX_POWER;
            double frontRightPower = (rotY - rotX - rx) / denominator * MAX_POWER;
            double backRightPower  = (rotY + rotX - rx) / denominator * MAX_POWER;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Heading", "%.1f°", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}
