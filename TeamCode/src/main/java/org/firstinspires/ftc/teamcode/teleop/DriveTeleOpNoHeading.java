package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Same as {@link DriveTeleOp} (field-centric, precision mode, stick snapping) but without heading hold,
 * so the robot only rotates when the driver turns it.
 */
@TeleOp(name = "Drive TeleOp (No Heading Hold)", group = "Competition")
public class DriveTeleOpNoHeading extends DriveTeleOp {

    @Override
    protected boolean headingHoldEnabled() {
        return false;
    }
}
