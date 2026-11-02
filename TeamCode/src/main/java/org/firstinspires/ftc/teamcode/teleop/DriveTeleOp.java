package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.List;

/**
 * Field-centric mecanum drive with heading hold, line hold and precision mode.
 * Uses the Pinpoint odometry when it's in the robot config (falls back to the hub IMU without line hold).
 *
 * Left stick            : move in a straight line (forward = away from driver, regardless of robot heading)
 * Right stick left/right: turn left / right (heading is held when released)
 * Right stick click    : reset "forward" to the direction the robot is facing
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
    // Hub's built-in IMU. Optional when the Pinpoint is present (its own IMU is used instead).
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

    // Minimum power each motor gets as soon as the DRIVER asks it to move, to overcome its friction so all
    // four wheels start together. Automatic corrections (heading hold, line hold, damping) never get it, so
    // they stay smooth. Raise one if that wheel still starts late; lower if the robot jumps at the first
    // touch of the stick. Actual motor power, not scaled by MAX_POWER.
    private static final double FRONT_LEFT_MIN_POWER  = 0.05;
    private static final double FRONT_RIGHT_MIN_POWER = 0.05;
    private static final double BACK_LEFT_MIN_POWER   = 0.05;
    private static final double BACK_RIGHT_MIN_POWER  = 0.05;
    // Driver commands smaller than this don't get the minimum power.
    private static final double MIN_POWER_THRESHOLD = 0.01;

    // Fraction of normal speed while precision mode is held.
    private static final double PRECISION_SCALE = 0.3;

    // How fast drive speed may rise / fall, in full-stick per second (3.0 = 0 to full in 0.33 s).
    // Sudden power changes make the rollers slip, which twists and slides the robot.
    // Lower for less slip, raise if the robot feels sluggish.
    private static final double ACCEL_RATE = 3.0;
    private static final double DECEL_RATE = 6.0;

    // ---------- Drift trims ----------
    // Constant corrections for this robot's own pull, so it drives straight even without heading hold.
    // Tune on "No Heading Hold" in normal mode, full stick, watching the °/s next to Heading:
    //   driving forward, robot turns left (°/s positive)  -> raise FORWARD_TURN_TRIM; turns right -> lower it.
    //   strafing right,  robot turns left (°/s positive)  -> raise STRAFE_TURN_TRIM;  turns right -> lower it.
    //   strafing right,  robot creeps forward             -> lower STRAFE_FORWARD_TRIM; creeps back -> raise it.
    // Change by about 0.02 at a time.
    private static final double FORWARD_TURN_TRIM   = 0.0;
    private static final double STRAFE_TURN_TRIM    = 0.0;
    private static final double STRAFE_FORWARD_TRIM = 0.0;

    // ---------- Heading hold ----------

    // Turn values below are before MAX_POWER scaling (actual motor power = value * MAX_POWER).
    // Turn power per degree of error. Raise if the robot still rotates while driving, lower if it wobbles.
    private static final double HEADING_KP       = 0.05;
    // Turn power per deg/s of rotation, pushing against rotation the moment it starts (before error builds).
    // Raise if it overshoots/wobbles back and forth.
    private static final double HEADING_KD       = 0.003;
    // Extra turn power added whenever the error is outside the tolerance, to overcome wheel friction
    // so even a 1-2° error gets corrected instead of being too weak to move the robot.
    private static final double HEADING_KS       = 0.06;
    // Errors smaller than this (degrees) count as on target, so the robot doesn't buzz in place.
    private static final double HEADING_TOLERANCE = 0.5;
    // HEADING_KS fades in over this many degrees past the tolerance instead of kicking in all at once,
    // so small errors don't cause twitching.
    private static final double HEADING_KS_FADE = 1.5;
    // Most turn power heading hold may use to correct.
    private static final double HEADING_MAX_TURN = 0.5;
    // After the turn stick is released, the heading locks once the robot spins slower than this (deg/s),
    // so it locks where the robot actually stops instead of snapping back to where it was mid-turn.
    private static final double HEADING_LOCK_RATE = 15.0;
    // Lock anyway after this long (s), in case the robot keeps rotating (e.g. being pushed).
    private static final double HEADING_LOCK_TIMEOUT = 0.6;

    // ---------- Odometry ----------

    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    // Pod positions from the robot's center of rotation, in inches. MEASURE THESE ON THIS ROBOT.
    // Forward pod: how far LEFT of center it is (right = negative).
    // Strafe pod:  how far FORWARD of center it is (behind = negative).
    private static final double FORWARD_POD_LEFT_IN  = 0.0;
    private static final double STRAFE_POD_FORWARD_IN = 0.0;
    // Flip if the X/Y telemetry counts the wrong way (push forward: X goes up; push left: Y goes up).
    private static final GoBildaPinpointDriver.EncoderDirection FORWARD_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private static final GoBildaPinpointDriver.EncoderDirection STRAFE_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Spin damping: while the turn stick is released and heading hold isn't locked, turn power per deg/s
    // of rotation. Brakes a turn the moment you let go so heading hold locks quickly with nothing to undo,
    // and is the only rotation correction in No Heading Hold. Raise if it twists, lower if it shakes.
    // Before MAX_POWER scaling.
    private static final double SPIN_DAMPING = 0.015;

    // Line hold: while the left stick points one way, the robot is pulled back onto the straight line it
    // started on. Drive power per inch off the line. Raise if it still slides off, lower if it weaves.
    private static final double LINE_KP = 0.05;
    // Most drive power line hold may use to correct.
    private static final double LINE_MAX_CORRECTION = 0.3;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    // Pinpoint heading (deg) that counts as field "forward"; set at start and by the reset button.
    private double headingOffset = 0.0;

    private boolean lineLocked = false;
    private double  lineAngle  = 0.0;
    private double  lineStartForward, lineStartStrafe;

    private boolean headingLocked = false;
    private double  targetHeading = 0.0;
    private final ElapsedTime sinceTurn = new ElapsedTime();

    private boolean lastResetButton = false;

    // Ramped drive command (field frame), and time since the last loop for ramping.
    private double rampedForward = 0.0;
    private double rampedStrafe  = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

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

        imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    HUB_LOGO_DIRECTION,
                    HUB_USB_DIRECTION
            )));
            imu.resetYaw();
        }

        // Found by type, so it works whatever the Pinpoint is named in the robot config.
        List<GoBildaPinpointDriver> pinpoints = hardwareMap.getAll(GoBildaPinpointDriver.class);
        pinpoint = pinpoints.isEmpty() ? null : pinpoints.get(0);
        if (pinpoint == null && imu == null) {
            throw new IllegalStateException("No Pinpoint or IMU named \"" + IMU_NAME + "\" in the robot config");
        }
        if (odometryEnabled()) {
            pinpoint.setOffsets(FORWARD_POD_LEFT_IN, STRAFE_POD_FORWARD_IN, DistanceUnit.INCH);
            pinpoint.setEncoderResolution(POD_TYPE);
            pinpoint.setEncoderDirections(FORWARD_POD_DIRECTION, STRAFE_POD_DIRECTION);
            pinpoint.resetPosAndIMU(); // robot must sit still for a moment after init
        }

        telemetry.addData("Odometry", odometryEnabled() ? "Pinpoint found"
                : "PINPOINT NOT FOUND - using hub IMU, no line hold");
        telemetry.addData("Status", "Ready - point robot away from driver before start");
        telemetry.update();
    }

    @Override
    public void start() {
        if (imu != null) imu.resetYaw();
        targetHeading = 0.0;
        headingLocked = true;
        loopTimer.reset();
        if (odometryEnabled()) {
            pinpoint.update();
            headingOffset = pinpoint.getHeading(AngleUnit.DEGREES);
        }
    }

    @Override
    public void loop() {
        // Reset field-centric forward.
        boolean resetButton = gamepad1.right_stick_button;
        if (odometryEnabled()) pinpoint.update();
        if (resetButton && !lastResetButton) {
            if (imu != null) imu.resetYaw();
            if (odometryEnabled()) headingOffset = pinpoint.getHeading(AngleUnit.DEGREES);
            targetHeading = 0.0;
            lineLocked = false;
            gamepad1.rumble(RESET_RUMBLE_MS);
        }
        lastResetButton = resetButton;

        double heading, yawRate;
        double fieldForward = 0.0, fieldStrafe = 0.0; // robot position (in) along field forward / right
        if (odometryEnabled()) {
            // Pinpoint's own IMU, so heading and position share one frame.
            heading = AngleUnit.normalizeDegrees(pinpoint.getHeading(AngleUnit.DEGREES) - headingOffset);
            yawRate = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            // Pinpoint X = forward, Y = left at its reset; rotate into the driver's field frame.
            double offsetRad = Math.toRadians(headingOffset);
            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);
            fieldForward =   x * Math.cos(offsetRad) + y * Math.sin(offsetRad);
            fieldStrafe  = -(-x * Math.sin(offsetRad) + y * Math.cos(offsetRad));
        } else {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        }

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
            if (!lineLocked || stickAngle != lineAngle) {
                lineLocked = true;
                lineAngle = stickAngle;
                lineStartForward = fieldForward;
                lineStartStrafe  = fieldStrafe;
            }
        } else {
            lineLocked = false;
        }

        double dt = loopTimer.seconds();
        loopTimer.reset();
        rampedForward = ramp(rampedForward, forward, dt);
        rampedStrafe  = ramp(rampedStrafe,  strafe,  dt);
        forward = rampedForward;
        strafe  = rampedStrafe;

        // Line hold: push back toward the line, only sideways to it so it never speeds up or slows down.
        double lineError = 0.0;
        if (odometryEnabled() && lineLocked) {
            double normalForward = -Math.sin(Math.toRadians(lineAngle));
            double normalStrafe  =  Math.cos(Math.toRadians(lineAngle));
            lineError = (fieldForward - lineStartForward) * normalForward
                      + (fieldStrafe  - lineStartStrafe)  * normalStrafe;
            double correction = Range.clip(-lineError * LINE_KP, -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);
            forward += correction * normalForward;
            strafe  += correction * normalStrafe;
        }

        double turn = Math.copySign(shapeInput(Math.abs(gamepad1.right_stick_x)), gamepad1.right_stick_x) * scale;

        // Spin damping: resist rotation the driver didn't ask for. Heading is CCW-positive; positive turn is clockwise.
        boolean driverTurning = turn != 0.0;
        if (odometryEnabled() && !driverTurning) {
            turn = Range.clip(yawRate * SPIN_DAMPING, -HEADING_MAX_TURN, HEADING_MAX_TURN);
        }

        // Heading hold
        if (driverTurning) {
            headingLocked = false;
            sinceTurn.reset();
        } else if (!headingLocked
                && (Math.abs(yawRate) < HEADING_LOCK_RATE || sinceTurn.seconds() > HEADING_LOCK_TIMEOUT)) {
            targetHeading = heading;
            headingLocked = true;
        }
        if (headingHoldEnabled() && headingLocked) {
            // Heading is CCW-positive; positive turn power turns clockwise.
            double error = AngleUnit.normalizeDegrees(targetHeading - heading);
            double correction = -error * HEADING_KP + yawRate * HEADING_KD;
            double ksFade = Range.clip((Math.abs(error) - HEADING_TOLERANCE) / HEADING_KS_FADE, 0.0, 1.0);
            correction += Math.copySign(HEADING_KS * ksFade, -error);
            turn = Range.clip(correction, -HEADING_MAX_TURN, HEADING_MAX_TURN);
        }

        // Field-centric: rotate stick input by the robot's heading
        double headingRad = Math.toRadians(heading);
        double robotStrafe  = (strafe * Math.cos(-headingRad) - forward * Math.sin(-headingRad)) * STRAFE_MULTIPLIER;
        double robotForward =  strafe * Math.sin(-headingRad) + forward * Math.cos(-headingRad);

        // Drift trims, in the robot's own frame since the pull comes from the robot itself.
        turn         += FORWARD_TURN_TRIM * robotForward + STRAFE_TURN_TRIM * robotStrafe;
        robotForward += STRAFE_FORWARD_TRIM * robotStrafe;

        // Scale down so no wheel exceeds full power while keeping the motion ratio.
        double denominator = Math.max(Math.abs(robotForward) + Math.abs(robotStrafe) + Math.abs(turn), 1.0);

        double k = MAX_POWER / denominator;
        // Driver's part of each wheel gets the minimum power; automatic turn corrections are added as-is.
        double driverTurn = driverTurning ? turn : 0.0;
        double autoTurn   = driverTurning ? 0.0 : turn;
        double fl = withMinPower((robotForward + robotStrafe + driverTurn) * k, FRONT_LEFT_MIN_POWER)  + autoTurn * k;
        double bl = withMinPower((robotForward - robotStrafe + driverTurn) * k, BACK_LEFT_MIN_POWER)   + autoTurn * k;
        double fr = withMinPower((robotForward - robotStrafe - driverTurn) * k, FRONT_RIGHT_MIN_POWER) - autoTurn * k;
        double br = withMinPower((robotForward + robotStrafe - driverTurn) * k, BACK_RIGHT_MIN_POWER)  - autoTurn * k;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);

        telemetry.addData("Mode", scale < 1.0 ? "PRECISION" : "Normal");
        if (!odometryEnabled()) telemetry.addData("Odometry", "PINPOINT NOT FOUND - using hub IMU, no line hold");
        telemetry.addData("Heading", "%.1f°  (%.0f°/s)", heading, yawRate);
        telemetry.addData("Heading Hold", !headingHoldEnabled() ? "DISABLED" : headingLocked ? String.format("ON (target %.1f°)", targetHeading) : "off");
        if (odometryEnabled()) {
            telemetry.addData("Odometry X / Y", "%.1f  %.1f in  (%s)",
                    pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getDeviceStatus());
            telemetry.addData("Line Hold", lineLocked ? String.format("ON (%.1f in off line)", lineError) : "off");
        }
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

    /** Whether the Pinpoint was found in the robot config, enabling spin damping and line hold. */
    private boolean odometryEnabled() {
        return pinpoint != null;
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
     * Lifts a motor command (-MAX_POWER to MAX_POWER) so any real command starts at minPower, while full
     * command still gives MAX_POWER. Commands below MIN_POWER_THRESHOLD are left unchanged.
     */
    private static double withMinPower(double power, double minPower) {
        if (Math.abs(power) < MIN_POWER_THRESHOLD) return power;
        double lifted = minPower + Math.abs(power) * (MAX_POWER - minPower) / MAX_POWER;
        return Math.copySign(Math.min(lifted, 1.0), power);
    }

    /** Moves current toward target, no faster than ACCEL_RATE when speeding up or DECEL_RATE when slowing down. */
    private static double ramp(double current, double target, double dt) {
        boolean slowingDown = Math.abs(target) < Math.abs(current) || target * current < 0.0;
        double maxStep = (slowingDown ? DECEL_RATE : ACCEL_RATE) * dt;
        return current + Range.clip(target - current, -maxStep, maxStep);
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
