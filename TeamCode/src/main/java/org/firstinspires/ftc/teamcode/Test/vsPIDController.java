package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class vsPIDController {
    //Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    // PID controller for handling the arm's motion - LC 12/9
    private final PIDController controller;
    // Motor controlling the viper slides - LC 12/9
    private final DcMotor lvsMotor;
    private final DcMotor rvsMotor;
    // Feedforward constant for gravity compensation - LC 12/9
    private final double feedforward;
    // Conversion factor from encoder ticks to degrees - LC 12/9
    private final double ticksPerDegree;

    /**
     * Constructor for the vsPIDController class. - LC 12/9
     *
     * @param lvsMotor  The motor controlling the vs thingy. - LC 12/9
     * @param p             The proportional gain for the PID controller. - LC 12/9
     * @param i             The integral gain for the PID controller. - LC 12/9
     * @param d             The derivative gain for the PID controller. - LC 12/9
     * @param f             The feedforward gain for gravity compensation. - LC 12/9
     * @param ticksPerDegree Conversion factor: encoder ticks per degree of rotation. - LC 12/9
     */
    public vsPIDController(DcMotor lvsMotor, DcMotor rvsMotor, double p, double i, double d, double f, double ticksPerDegree) {
        this.controller = new PIDController(p, i, d);
        this.lvsMotor = lvsMotor;
        this.rvsMotor = rvsMotor;
        this.feedforward = f;
        this.ticksPerDegree = ticksPerDegree;
    }
    /**
     * Updates the PID coefficients dynamically. - LC 12/9
     *
     * @param p The new proportional gain. - LC 12/9
     * @param i The new integral gain. - LC 12/9
     * @param d The new derivative gain. - LC 12/9
     */
    public void setPID(double p, double i, double d) {
        controller.setPID(p, i, d);
    }

    /**
     * Sets the target position for the arm motor. - LC 12/9
     *
     * @param targetPosition The desired encoder position for the arm/joint motor. - LC 12/9
     */
    public void setTarget(int targetPosition) {
        controller.setSetPoint(targetPosition);
    }

    /*
     * Updates the motor power based on the PID calculation and feedforward term. - LC 12/9
     * This method should be called periodically in the control loop. - LC 12/9
     */
    public void update() {
        int armPos = lvsMotor.getCurrentPosition();
        double pid = controller.calculate(armPos);
        double ff = Math.cos(Math.toRadians(controller.getSetPoint() / ticksPerDegree)) * feedforward;
        double power = pid + ff;

        lvsMotor.setPower(power);
        rvsMotor.setPower(-power);

    }
    public int getTarget() {
        return (int) controller.getSetPoint(); // Assuming `controller` has a `getSetPoint` method
    }
}

