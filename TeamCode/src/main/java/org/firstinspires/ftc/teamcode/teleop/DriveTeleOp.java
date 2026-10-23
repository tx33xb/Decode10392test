package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric mecanum drive with heading hold and precision mode.
 *
 * Left stick            : move (forward = away from driver, regardless of robot heading)
 * Right stick left/right: turn left / right (heading is held when released)
 * Right stick click     : reset "forward" to the direction the robot is facing
 * Hold left trigger     : precision mode (slow)
 */
@TeleOp(name = "Drive TeleOp", group = "Competition")
public class DriveTeleOp extends OpMode {

    private static final double DEADZONE = 0.05;

    // Compensates for mecanum strafing being slightly weaker than driving forward.
    private static final double STRAFE_MULTIPLIER = 1.1;

    private static final double PRECISION_SCALE = 0.3;

    // Heading hold: turn power per degree of error. Raise if the robot drifts, lower if it wobbles.
    private static final double HEADING_KP       = 0.025;
    private static final double HEADING_MAX_TURN = 0.4;
    // Wait after releasing the turn stick so the robot finishes turning before the heading is locked.
    private static final double HEADING_LOCK_DELAY = 0.25;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private boolean headingLocked = false;
    private double  targetHeading = 0.0;
    private final ElapsedTime sinceTurn = new ElapsedTime();

    private boolean lastResetButton = false;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight  = hardwareMap.get(DcMotor.class, "back_right_motor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
        imu.resetYaw();

        telemetry.addData("Status", "Ready - point robot away from driver before start");
        telemetry.update();
    }

    @Override
    public void start() {
        imu.resetYaw();
        targetHeading = 0.0;
        headingLocked = true;
    }

    @Override
    public void loop() {
        // Reset field-centric forward
        boolean resetButton = gamepad1.right_stick_button;
        if (resetButton && !lastResetButton) {
            imu.resetYaw();
            targetHeading = 0.0;
            gamepad1.rumble(200);
        }
        lastResetButton = resetButton;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double scale   = gamepad1.left_trigger > 0.5 ? PRECISION_SCALE : 1.0;
        double forward = deadzone(-gamepad1.left_stick_y) * scale;
        double strafe  = deadzone(gamepad1.left_stick_x)  * scale;
        double turn    = deadzone(gamepad1.right_stick_x) * scale;

        // Heading hold
        if (turn != 0.0) {
            headingLocked = false;
            sinceTurn.reset();
        } else if (!headingLocked && sinceTurn.seconds() > HEADING_LOCK_DELAY) {
            targetHeading = heading;
            headingLocked = true;
        }
        if (headingLocked) {
            // Heading is CCW-positive; positive turn power turns clockwise.
            double error = AngleUnit.normalizeDegrees(targetHeading - heading);
            turn = Range.clip(-error * HEADING_KP, -HEADING_MAX_TURN, HEADING_MAX_TURN);
        }

        // Field-centric: rotate stick input by the robot's heading
        double headingRad = Math.toRadians(heading);
        double robotStrafe  = (strafe * Math.cos(-headingRad) - forward * Math.sin(-headingRad)) * STRAFE_MULTIPLIER;
        double robotForward =  strafe * Math.sin(-headingRad) + forward * Math.cos(-headingRad);

        // Scale down so no wheel exceeds full power while keeping the motion ratio.
        double denominator = Math.max(Math.abs(robotForward) + Math.abs(robotStrafe) + Math.abs(turn), 1.0);

        double fl = (robotForward + robotStrafe + turn) / denominator;
        double bl = (robotForward - robotStrafe + turn) / denominator;
        double fr = (robotForward - robotStrafe - turn) / denominator;
        double br = (robotForward + robotStrafe - turn) / denominator;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);

        telemetry.addData("Mode", scale < 1.0 ? "PRECISION" : "Normal");
        telemetry.addData("Heading", "%.1f°", heading);
        telemetry.addData("Heading Hold", headingLocked ? String.format("ON (target %.1f°)", targetHeading) : "off");
        telemetry.addData("FL / FR", "%.2f  %.2f", fl, fr);
        telemetry.addData("BL / BR", "%.2f  %.2f", bl, br);
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private static double deadzone(double value) {
        return Math.abs(value) < DEADZONE ? 0.0 : value;
    }
}
