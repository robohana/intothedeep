package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A controller for managing a DcMotor used as a hiExtend mechanism. - LC 12/10
 * Provides methods to extend, retract, and stop the hiExtend operation. - LC 12/10
 */
@Config
public class hiExtendController {

    public DcMotor hiExtend;

    // Define the power levels for extend and retract - LC 12/10
    private static final double EXTEND_POWER = 1.0;  // Power to extend the arm - LC 12/10
    private static final double RETRACT_POWER = -1.0;  // Power to retract the arm - LC 12/10
    private static final double STOP_POWER = 0.0;     // Power to stop the arm - LC 12/10

    /**
     * Constructor for the hiExtendController. - LC 12/10
     *
     * @param hiExtend The DcMotor controlling the hiExtend. - LC 12/10
     */
    public hiExtendController(DcMotor hiExtend) {
        this.hiExtend = hiExtend;
    }
    // Activates the hiExtend to extend the arm using EXTEND_POWER - LC 12/10
    public void extend() {
        hiExtend.setPower(EXTEND_POWER);
    }
    // Activates the hiExtend to retract the arm using RETRACT_POWER - LC 12/10
    public void retract() {
        hiExtend.setPower(RETRACT_POWER);
    }
    // Stops the arm by setting the DcMotor power to STOP_POWER - LC 12/10
    public void stop() {
        hiExtend.setPower(STOP_POWER);
    }
}
