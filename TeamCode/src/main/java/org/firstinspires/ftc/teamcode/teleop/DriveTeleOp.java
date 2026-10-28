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
 * Share / Back button   : reset "forward" to the direction the robot is facing
 * Hold left trigger     : precision mode (slow)
 */
@TeleOp(name = "Drive TeleOp", group = "Competition")
public class DriveTeleOp extends OpMode {

    // ---------- Hardware ----------

    // Names must match the active robot configuration on the Driver Station.
    private static final String FRONT_LEFT_NAME  = "fl_motor";
    private static final String FRONT_RIGHT_NAME = "front_right_motor";
    private static final String BACK_LEFT_NAME   = "back_left_motor";
    private static final String BACK_RIGHT_NAME  = "back_right_motor";
    private static final String IMU_NAME         = "imu";

    // How the Control Hub is mounted. If these are wrong, field-centric and heading hold misbehave.
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // ---------- Driver input ----------

    // Stick travel ignored around center so a resting stick doesn't creep the robot.
    private static final double DEADZONE = 0.05;

    // Stick response curve: 1.0 = linear, 2.0 = squared (finer control at low speed).
    private static final double INPUT_EXPONENT = 2.0;

    // Left stick within this many degrees of straight forward/back/left/right snaps exactly onto it;
    // anything further off snaps to the nearest 45° diagonal. 45 removes diagonals entirely.
    private static final double STRAIGHT_SNAP_DEG = 30.0;

    // How far the left trigger must be pressed to enter precision mode.
    private static final double PRECISION_TRIGGER_THRESHOLD = 0.5;

    // Rumble length (ms) when field-centric forward is reset.
    private static final int RESET_RUMBLE_MS = 200;

    // ---------- Drive output ----------

    // Compensates for mecanum strafing being slightly weaker than driving forward.
    private static final double STRAFE_MULTIPLIER = 1.1;

    // Highest power any drive motor is given (1.0 = full power).
    private static final double MAX_POWER = 0.5;

    // Fraction of normal speed while precision mode is held.
    private static final double PRECISION_SCALE = 0.3;

    // ---------- Heading hold ----------

    // Turn power per degree of error. Raise if the robot drifts, lower if it wobbles.
    private static final double HEADING_KP       = 0.025;
    // Most turn power heading hold may use to correct.
    private static final double HEADING_MAX_TURN = 0.4;
    // Wait (s) after releasing the turn stick so the robot finishes turning before the heading is locked.
    private static final double HEADING_LOCK_DELAY = 0.25;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private boolean headingLocked = false;
    private double  targetHeading = 0.0;
    private final ElapsedTime sinceTurn = new ElapsedTime();

    private boolean lastResetButton = false;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft   = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight  = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                HUB_LOGO_DIRECTION,
                HUB_USB_DIRECTION
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
        // Reset field-centric forward. Kept off the sticks so pushing the turn stick can't reset it by accident.
        boolean resetButton = gamepad1.back;
        if (resetButton && !lastResetButton) {
            imu.resetYaw();
            targetHeading = 0.0;
            gamepad1.rumble(RESET_RUMBLE_MS);
        }
        lastResetButton = resetButton;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double scale = gamepad1.left_trigger > PRECISION_TRIGGER_THRESHOLD ? PRECISION_SCALE : 1.0;

        // Left stick: speed comes from how far it's pushed, direction from its angle.
        double stickForward = -gamepad1.left_stick_y;
        double stickStrafe  =  gamepad1.left_stick_x;
        double driveSpeed   = shapeInput(Math.hypot(stickForward, stickStrafe)) * scale;
        double forward = 0.0;
        double strafe  = 0.0;
        if (driveSpeed > 0.0) {
            // 0° = forward, 90° = right, -90° = left, ±180° = backward.
            double stickAngle = snapToDirection(Math.toDegrees(Math.atan2(stickStrafe, stickForward)));
            forward = driveSpeed * Math.cos(Math.toRadians(stickAngle));
            strafe  = driveSpeed * Math.sin(Math.toRadians(stickAngle));
        }

        double turn = Math.copySign(shapeInput(Math.abs(gamepad1.right_stick_x)), gamepad1.right_stick_x) * scale;

        // Heading hold
        if (turn != 0.0) {
            headingLocked = false;
            sinceTurn.reset();
        } else if (!headingLocked && sinceTurn.seconds() > HEADING_LOCK_DELAY) {
            targetHeading = heading;
            headingLocked = true;
        }
        if (headingHoldEnabled() && headingLocked) {
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

        double fl = (robotForward + robotStrafe + turn) / denominator * MAX_POWER;
        double bl = (robotForward - robotStrafe + turn) / denominator * MAX_POWER;
        double fr = (robotForward - robotStrafe - turn) / denominator * MAX_POWER;
        double br = (robotForward + robotStrafe - turn) / denominator * MAX_POWER;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);

        telemetry.addData("Mode", scale < 1.0 ? "PRECISION" : "Normal");
        telemetry.addData("Heading", "%.1f°", heading);
        telemetry.addData("Heading Hold", !headingHoldEnabled() ? "DISABLED" : headingLocked ? String.format("ON (target %.1f°)", targetHeading) : "off");
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

    /** Whether heading hold corrects unwanted rotation. {@link DriveTeleOpNoHeading} turns it off. */
    protected boolean headingHoldEnabled() {
        return true;
    }

    /**
     * Turns a stick push (0 to 1) into a speed (0 to 1). Pushes inside the deadzone give 0, and the
     * rest of the travel is stretched to start smoothly from 0 instead of jumping to the deadzone value.
     */
    private static double shapeInput(double push) {
        push = Math.min(push, 1.0);
        if (push < DEADZONE) return 0.0;
        double rescaled = (push - DEADZONE) / (1.0 - DEADZONE);
        return Math.pow(rescaled, INPUT_EXPONENT);
    }

    /**
     * Snaps a stick angle to straight forward/back/left/right when within STRAIGHT_SNAP_DEG of it,
     * otherwise to the 45° diagonal between the two neighbouring straight directions.
     */
    private static double snapToDirection(double angleDeg) {
        double nearestStraight = Math.round(angleDeg / 90.0) * 90.0;
        if (Math.abs(angleDeg - nearestStraight) <= STRAIGHT_SNAP_DEG) return nearestStraight;
        return Math.floor(angleDeg / 90.0) * 90.0 + 45.0;
    }
}
