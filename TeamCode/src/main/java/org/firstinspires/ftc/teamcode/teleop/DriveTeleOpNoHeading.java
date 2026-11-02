package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Same as {@link DriveTeleOp} (field-centric, line hold, precision mode, stick snapping) but without heading
 * hold: rotation is only resisted by spin damping, never turned back to a saved heading.
 */
@TeleOp(name = "Drive TeleOp (No Heading Hold)", group = "Competition")
public class DriveTeleOpNoHeading extends DriveTeleOp {

    @Override
    protected boolean headingHoldEnabled() {
        return false;
    }
}
