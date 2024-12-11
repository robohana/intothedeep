package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * A controller for managing a continuous rotation servo used as a claw mechanism. - LC 12/10
 * Provides methods to open, close, and stop the claw operation. - LC 12/10
 */
@Config
public class ClawController {

    private final CRServo clawServo;

    // Define the power levels for open and close - LC 12/10
    private static final double OPEN_POWER = -1.0;  // Power to open the claw - LC 12/10
    private static final double CLOSE_POWER = 1.0;  // Power to close the claw - LC 12/10
    private static final double STOP_POWER = 0.0;   // Power to stop the claw - LC 12/10

    /**
     * Constructor for the ClawController. - LC 12/10
     *
     * @param clawServo The CRServo controlling the claw. - LC 12/10
     */
    public ClawController(CRServo clawServo) {
        this.clawServo = clawServo;
    }

    // Opens the claw by setting the servo power to OPEN_POWER - LC 12/10
    public void openClaw() {
        clawServo.setPower(OPEN_POWER);
    }

    // Closes the claw by setting the servo power to CLOSE_POWER - LC 12/10
    public void closeClaw() {
        clawServo.setPower(CLOSE_POWER);
    }

    // Stops the claw by setting the servo power to STOP_POWER - LC 12/10
    public void stopClaw() {
        clawServo.setPower(STOP_POWER);
    }
}

