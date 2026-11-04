package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Field-centric mecanum drive with heading hold, line hold and precision mode.
 * Heading and position come only from the Pinpoint odometry computer (and its own IMU); the Control Hub's
 * built-in IMU is never used. Pinpoint setup and the correction gains come from the Pedro autotune results
 * in {@link Constants}.
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
    // Pinpoint (Control Hub I2C bus 1) name, type, offsets and directions are read from
    // Constants.localizerConfig, so re-running the Pedro tuner and pasting the new Constants updates this too.

    // ---------- Driver input ----------

    // Stick travel ignored around center so a resting stick doesn't creep the robot.
    private static final double DEADZONE = 0.05;

    // Stick response curve: 1.0 = linear, 2.0 = squared (finer control at low speed).
    private static final double INPUT_EXPONENT = 2.0;

    // Left stick within this many degrees of straight forward/back/left/right snaps exactly onto it;
    // anything further off snaps to the nearest 45° diagonal. 45 removes diagonals entirely.
    // Line hold holds the snapped direction.
    private static final double STRAIGHT_SNAP_DEG = 30.0;

    // How far the left trigger must be pressed to enter precision mode.
    private static final double PRECISION_TRIGGER_THRESHOLD = 0.5;

    // Rumble length (ms) when field-centric forward is reset.
    private static final int RESET_RUMBLE_MS = 200;

    // ---------- Drive output ----------

    // Compensates for mecanum strafing being weaker than driving forward: Pedro autotune measured
    // 84.0 in/s top speed forward vs 71.1 in/s strafing (maxAchievable*Velocity in Constants).
    private static final double STRAFE_MULTIPLIER = 84.02 / 71.07;

    // Highest power any drive motor is given (1.0 = full power).
    private static final double MAX_POWER = 0.5;

    // Fraction of normal speed while precision mode is held.
    private static final double PRECISION_SCALE = 0.3;

    // ---------- Heading hold ----------

    // Turn values below are before MAX_POWER scaling (actual motor power = value * MAX_POWER).
    // Turn power per degree of error. Raise if the robot still rotates while driving, lower if it wobbles.
    // Pedro autotune headingFeedback: 3.74 motor power per radian = 0.0653 per degree.
    private static final double HEADING_KP       = 0.0653 / MAX_POWER;
    // Turn power per deg/s of rotation, pushing against rotation the moment it starts (before error builds).
    // Raise if it overshoots/wobbles back and forth (e.g. as it stops); lower if it buzzes.
    private static final double HEADING_KD       = 0.009;
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
    // When heading hold locks within this many degrees of 0°, 90°, 180° or -90°, it holds exactly that angle,
    // so the robot stays square to the field. 0 = always hold wherever the robot stopped.
    private static final double HEADING_SQUARE_SNAP_DEG = 5.0;
    // A small error that never goes away (e.g. the drivetrain always twisting one way) is added up over time and
    // pushed out within about this many seconds. Shorter = removes it faster, too short = wobble.
    private static final double HEADING_INTEGRAL_TIME = 0.5;
    // Most extra error (degrees) the added-up part may add.
    private static final double HEADING_MAX_INTEGRAL = 3.0;

    // ---------- Odometry corrections ----------

    // Spin damping: while the turn stick is released and heading hold isn't locked, turn power per deg/s
    // of rotation. Brakes a turn the moment you let go so heading hold locks quickly with nothing to undo,
    // and is the only rotation correction in No Heading Hold. Raise if it twists, lower if it shakes.
    // Before MAX_POWER scaling.
    private static final double SPIN_DAMPING = 0.015;

    // Line hold: while the left stick points one way, the robot is pulled back onto the straight line it
    // started on. Drive power per inch off the line, split into the robot's own forward and strafe directions
    // since strafing needs more power. Pedro autotune small-error translational gains (secondary, < 2.5 in):
    // 0.0705 motor power per inch forward, 0.100 per inch strafing. Lower both if it weaves.
    private static final double LINE_KP_FORWARD = 0.0705 / MAX_POWER;
    private static final double LINE_KP_STRAFE  = 0.1003 / MAX_POWER;
    // Most drive power line hold may use to correct, in each direction.
    private static final double LINE_MAX_CORRECTION = 0.3;
    // Line hold corrects where the robot will be this many seconds from now at its current sideways speed,
    // so it pushes back the moment a slide starts instead of waiting for the error to build up.
    // Raise if it still slides off before correcting, lower if it shakes side to side.
    private static final double LINE_LOOKAHEAD = 0.1;
    // After the left stick is released, line hold keeps working until the robot slows below this (in/s),
    // so it doesn't slide sideways while stopping.
    private static final double LINE_RELEASE_SPEED = 3.0;
    // LINE_LOOKAHEAD used while stopping. Longer = more braking of the sideways slide and less pulling back,
    // so the robot doesn't wiggle as it stops. Raise if it still wiggles sideways, lower if it stops off the line.
    private static final double LINE_STOP_LOOKAHEAD = 0.3;
    // A steady pull off the line (e.g. one side of the drivetrain stronger) leaves a small offset the gains alone
    // can't remove; it's added up over time and pushed out within about this many seconds.
    // Shorter = gets back onto the line faster, too short = weaves.
    private static final double LINE_INTEGRAL_TIME = 0.5;
    // Most extra error (in) the added-up part may add.
    private static final double LINE_MAX_INTEGRAL = 2.0;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;

    // Pinpoint heading (deg) that counts as field "forward"; set at start and by the reset button.
    private double headingOffset = 0.0;

    private boolean lineLocked = false;
    private double  lineStickAngle = 0.0; // snapped stick angle the line was started with
    private double  lineAngle  = 0.0;     // field direction of the line (deg, 0 = field forward, 90 = right)
    private double  lineStartForward, lineStartStrafe;
    private double  lineIntegral = 0.0;   // inch-seconds off the line

    private boolean headingLocked = false;
    private double  targetHeading = 0.0;
    private double  headingIntegral = 0.0; // degree-seconds off target
    private final ElapsedTime sinceTurn = new ElapsedTime();

    private boolean lastResetButton = false;
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

        String pinpointName = Constants.localizerConfig.name.get();
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, pinpointName);
        if (pinpoint == null) {
            throw new IllegalStateException("No goBILDA Pinpoint named \"" + pinpointName
                    + "\" in the robot config (should be on Control Hub I2C bus 1)");
        }
        // Same setup Pedro's PinpointLocalizer does.
        pinpoint.setOffsets(Constants.localizerConfig.xPodOffset.get(), Constants.localizerConfig.yPodOffset.get(),
                Constants.localizerConfig.offsetUnits.get());
        pinpoint.setEncoderResolution(Constants.localizerConfig.podType.get());
        pinpoint.setEncoderDirections(Constants.localizerConfig.xPodDirection.get(),
                Constants.localizerConfig.yPodDirection.get());
        pinpoint.resetPosAndIMU(); // robot must sit still for a moment after init

        telemetry.addData("Status", "Ready - point robot away from driver before start");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Status", "Ready - point robot away from driver before start");
        telemetry.update();
    }

    @Override
    public void start() {
        targetHeading = 0.0;
        headingLocked = true;
        headingIntegral = 0.0;
        loopTimer.reset();
        pinpoint.update();
        headingOffset = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    @Override
    public void loop() {
        pinpoint.update();
        // Loop time for the added-up corrections; capped so one slow loop can't add a big jump.
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        // Reset field-centric forward.
        boolean resetButton = gamepad1.right_stick_button;
        if (resetButton && !lastResetButton) {
            headingOffset = pinpoint.getHeading(AngleUnit.DEGREES);
            targetHeading = 0.0;
            headingIntegral = 0.0;
            lineLocked = false;
            gamepad1.rumble(RESET_RUMBLE_MS);
        }
        lastResetButton = resetButton;

        // Pinpoint's own IMU, so heading and position share one frame.
        double heading = AngleUnit.normalizeDegrees(pinpoint.getHeading(AngleUnit.DEGREES) - headingOffset);
        double yawRate = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        // Robot position (in) along field forward / right. Pinpoint X = forward, Y = left at its reset;
        // rotate into the driver's field frame.
        double offsetRad = Math.toRadians(headingOffset);
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double fieldForward =   x * Math.cos(offsetRad) + y * Math.sin(offsetRad);
        double fieldStrafe  = -(-x * Math.sin(offsetRad) + y * Math.cos(offsetRad));
        // Robot velocity (in/s) in the same field frame.
        double vx = pinpoint.getVelX(DistanceUnit.INCH);
        double vy = pinpoint.getVelY(DistanceUnit.INCH);
        double fieldVelForward =   vx * Math.cos(offsetRad) + vy * Math.sin(offsetRad);
        double fieldVelStrafe  = -(-vx * Math.sin(offsetRad) + vy * Math.cos(offsetRad));
        double speed = Math.hypot(vx, vy);

        double scale = gamepad1.left_trigger > PRECISION_TRIGGER_THRESHOLD ? PRECISION_SCALE : 1.0;

        double turn = Math.copySign(shapeInput(Math.abs(gamepad1.right_stick_x)), gamepad1.right_stick_x) * scale;
        boolean driverTurning = turn != 0.0;

        // Left stick: speed comes from how far it's pushed, direction from its angle.
        double stickForward = -gamepad1.left_stick_y;
        double stickStrafe  =  gamepad1.left_stick_x;
        double driveSpeed   = shapeInput(Math.hypot(stickForward, stickStrafe)) * scale;
        double forward = 0.0;
        double strafe  = 0.0;
        if (driveSpeed > 0.0) {
            // 0° = forward, 90° = right, -90° = left, ±180° = backward.
            double stickAngle = snapToDirection(Math.toDegrees(Math.atan2(stickStrafe, stickForward)));
            // Robot-centric: the stick is relative to the robot, so a new line starts whenever the driver turns,
            // and the line is locked in the field direction the robot was pointing that way when it started.
            // While heading is held, the held heading is used instead of the measured one, so a fraction of a
            // degree of wobble at that moment doesn't angle the whole line.
            if (!lineLocked || stickAngle != lineStickAngle || (!fieldCentric() && driverTurning)) {
                double robotHeading = headingHoldEnabled() && headingLocked ? targetHeading : heading;
                lineLocked = true;
                lineStickAngle = stickAngle;
                lineAngle = fieldCentric() ? stickAngle : AngleUnit.normalizeDegrees(stickAngle - robotHeading);
                lineStartForward = fieldForward;
                lineStartStrafe  = fieldStrafe;
                lineIntegral = 0.0;
            }
            // Field direction to drive (0° = field forward, 90° = field right).
            forward = driveSpeed * Math.cos(Math.toRadians(lineAngle));
            strafe  = driveSpeed * Math.sin(Math.toRadians(lineAngle));
        } else if (speed < LINE_RELEASE_SPEED) {
            // Stick released: keep the line while the robot is still rolling, drop it once it has stopped.
            lineLocked = false;
        }

        // Line hold: how far (in) the robot is (about to be) off the line, measured sideways to it so it never speeds
        // up or slows down. The push back is applied after the field-centric rotation, using the robot-frame gains.
        double offLine = 0.0;   // actual distance off the line (in), for telemetry
        double lineError = 0.0; // what line hold corrects
        double normalForward = 0.0, normalStrafe = 0.0;
        if (lineLocked) {
            normalForward = -Math.sin(Math.toRadians(lineAngle));
            normalStrafe  =  Math.cos(Math.toRadians(lineAngle));
            offLine = (fieldForward - lineStartForward) * normalForward
                    + (fieldStrafe  - lineStartStrafe)  * normalStrafe;
            // Only small, lasting offsets are added up while driving; a shove is handled by the normal correction.
            // Dropped once the stick is released, since the pull it cancels fades as the robot stops.
            if (driveSpeed == 0.0) {
                lineIntegral = 0.0;
            } else if (Math.abs(offLine) < LINE_MAX_INTEGRAL) {
                double maxIntegral = LINE_MAX_INTEGRAL * LINE_INTEGRAL_TIME;
                lineIntegral = Range.clip(lineIntegral + offLine * dt, -maxIntegral, maxIntegral);
            }
            double sideVel = fieldVelForward * normalForward + fieldVelStrafe * normalStrafe;
            if (driveSpeed == 0.0) {
                // Stopping: extra-damped pull back, so the slide is stopped without swinging side to side.
                lineError = offLine + sideVel * LINE_STOP_LOOKAHEAD;
            } else {
                lineError = offLine + sideVel * LINE_LOOKAHEAD + lineIntegral / LINE_INTEGRAL_TIME;
            }
        }

        // Spin damping: resist rotation the driver didn't ask for. Heading is CCW-positive; positive turn is clockwise.
        if (!driverTurning) {
            turn = Range.clip(yawRate * SPIN_DAMPING, -HEADING_MAX_TURN, HEADING_MAX_TURN);
        }

        // Heading hold
        if (driverTurning) {
            headingLocked = false;
            sinceTurn.reset();
        } else if (!headingLocked
                && (Math.abs(yawRate) < HEADING_LOCK_RATE || sinceTurn.seconds() > HEADING_LOCK_TIMEOUT)) {
            targetHeading = heading;
            double square = Math.round(heading / 90.0) * 90.0;
            if (Math.abs(heading - square) <= HEADING_SQUARE_SNAP_DEG) {
                targetHeading = AngleUnit.normalizeDegrees(square);
            }
            headingLocked = true;
            headingIntegral = 0.0;
            // Robot-centric: restart the line from the exact held heading if driving began while still settling.
            if (!fieldCentric() && headingHoldEnabled() && driveSpeed > 0.0) lineLocked = false;
        }
        if (headingHoldEnabled() && headingLocked) {
            // Heading is CCW-positive; positive turn power turns clockwise.
            double error = AngleUnit.normalizeDegrees(targetHeading - heading);
            // Only small, lasting errors are added up while driving, so turning onto a square angle or a shove
            // doesn't overshoot. Dropped once the left stick is released: the twist it cancels comes from driving,
            // so keeping it would twist the robot the other way as it stops.
            if (driveSpeed == 0.0) {
                headingIntegral = 0.0;
            } else if (Math.abs(error) < HEADING_MAX_INTEGRAL) {
                double maxIntegral = HEADING_MAX_INTEGRAL * HEADING_INTEGRAL_TIME;
                headingIntegral = Range.clip(headingIntegral + error * dt, -maxIntegral, maxIntegral);
            }
            double correction = -(error + headingIntegral / HEADING_INTEGRAL_TIME) * HEADING_KP + yawRate * HEADING_KD;
            double ksFade = Range.clip((Math.abs(error) - HEADING_TOLERANCE) / HEADING_KS_FADE, 0.0, 1.0);
            correction += Math.copySign(HEADING_KS * ksFade, -error);
            turn = Range.clip(correction, -HEADING_MAX_TURN, HEADING_MAX_TURN);
        }

        // Field-centric: rotate stick input by the robot's heading
        double headingRad = Math.toRadians(heading);
        double robotStrafe  = (strafe * Math.cos(-headingRad) - forward * Math.sin(-headingRad)) * STRAFE_MULTIPLIER;
        double robotForward =  strafe * Math.sin(-headingRad) + forward * Math.cos(-headingRad);

        // Line hold push back, rotated into the robot frame (strafe gain already covers strafing being weaker).
        if (lineError != 0.0) {
            double backForward = -lineError * normalForward;
            double backStrafe  = -lineError * normalStrafe;
            double backRobotStrafe  = backStrafe * Math.cos(-headingRad) - backForward * Math.sin(-headingRad);
            double backRobotForward = backStrafe * Math.sin(-headingRad) + backForward * Math.cos(-headingRad);
            robotForward += Range.clip(backRobotForward * LINE_KP_FORWARD, -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);
            robotStrafe  += Range.clip(backRobotStrafe  * LINE_KP_STRAFE,  -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);
        }

        // Scale down so no wheel exceeds full power while keeping the motion ratio.
        double denominator = Math.max(Math.abs(robotForward) + Math.abs(robotStrafe) + Math.abs(turn), 1.0);
        double k = MAX_POWER / denominator;
        double fl = (robotForward + robotStrafe + turn) * k;
        double bl = (robotForward - robotStrafe + turn) * k;
        double fr = (robotForward - robotStrafe - turn) * k;
        double br = (robotForward + robotStrafe - turn) * k;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);

        telemetry.addData("Mode", scale < 1.0 ? "PRECISION" : "Normal");
        telemetry.addData("Heading", "%.1f°  (%.0f°/s)", heading, yawRate);
        telemetry.addData("Heading Hold", !headingHoldEnabled() ? "DISABLED" : headingLocked ? String.format("ON (target %.1f°)", targetHeading) : "off");
        telemetry.addData("Odometry X / Y", "%.1f  %.1f in  (%s)", x, y, pinpoint.getDeviceStatus());
        telemetry.addData("Line Hold", lineLocked ? String.format("ON (%.2f in off line)", offLine) : "off");
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
     * Whether the left stick is relative to the field (true) or to the robot's front (false).
     * {@link RobotCentricDriveTeleOp} turns it off.
     */
    protected boolean fieldCentric() {
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
