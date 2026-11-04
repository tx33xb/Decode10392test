package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Same as {@link DriveTeleOp} (heading hold, line hold, spin damping, precision mode, stick snapping) but
 * robot-centric: pushing the left stick forward always drives toward the robot's front, whichever way it faces.
 * The Pinpoint still holds the heading and keeps the robot on a straight line, so it drives without drift.
 *
 * Left stick            : move in a straight line relative to the robot's front
 * Right stick left/right: turn left / right (heading is held when released)
 * Hold left trigger     : precision mode (slow)
 */
@TeleOp(name = "Drive TeleOp (Robot Centric)", group = "Competition")
public class RobotCentricDriveTeleOp extends DriveTeleOp {

    @Override
    protected boolean fieldCentric() {
        return false;
    }
}
