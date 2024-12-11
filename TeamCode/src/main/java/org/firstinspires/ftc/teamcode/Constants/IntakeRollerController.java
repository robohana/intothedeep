package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * A controller for managing a continuous rotation servo used as a intake mechanism. - LC 12/10
 * Provides methods to intake, release, and stop the intakeRoller operation. - LC 12/10
 */
@Config
public class IntakeRollerController {

    private final CRServo intakeRoller;

    // Define the power levels for intake and release - LC 12/10
    private static final double INTAKE_POWER = -1.0;  // Power for intaking objects - LC 12/10
    private static final double RELEASE_POWER = 1.0;  // Power for releasing objects - LC 12/10
    private static final double STOP_POWER = 0.0;     // Power to stop the roller - LC 12/10

    /**
     * Constructor for the IntakerollerController. - LC 12/10
     *
     * @param intakeRoller The CRServo controlling the intakeRoller. - LC 12/10
     */
    public IntakeRollerController(CRServo intakeRoller) {
        this.intakeRoller = intakeRoller;
    }

    // Activates the intake roller to pull objects in using INTAKE_POWER - LC 12/10
    public void intake() {
        intakeRoller.setPower(INTAKE_POWER);
    }

    // Activates the intake roller to push objects out using RELEASE_POWER - LC 12/10
    public void release() {
        intakeRoller.setPower(RELEASE_POWER);
    }

    // Stops the intake roller by setting the servo power to STOP_POWER - LC 12/10
    public void stopintake() {
        intakeRoller.setPower(STOP_POWER);
    }
}
