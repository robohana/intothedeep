package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name = "PIDvs")
public class PIDvs extends OpMode {
    private PIDController controller;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.1;

    public static int target = 0;

    private final double ticks_in_degree = 537.7 / 180.0;

    public DcMotor leftviperSlide;
    public DcMotor rightviperSlide;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftviperSlide = hardwareMap.get(DcMotor.class, "leftviperSlide");
        rightviperSlide = hardwareMap.get(DcMotor.class, "rightviperSlide");

        // Reset encoders to ensure consistent readings
        leftviperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightviperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftviperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightviperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





    }

    @Override
    public void loop() {
        // Update PID coefficients from dashboard
        controller.setPID(p, i, d);

        // Get the position from one motor (since they're mechanically linked)
        int currentPosition = leftviperSlide.getCurrentPosition();
        int rcurrentPosition = leftviperSlide.getCurrentPosition();


        // Calculate PID output
        double pid = controller.calculate(currentPosition, target);

        // Add feedforward term
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        // Calculate final power
        double power = pid + ff;

        // Set both motors with opposite directions
        leftviperSlide.setPower(power);
        rightviperSlide.setPower(-power);

        // Telemetry for debugging
        telemetry.addData("Left Position", currentPosition);
        telemetry.addData("Right Position", rcurrentPosition);

        telemetry.addData("Target", target);
        telemetry.addData("Power", power);
        telemetry.update();
    }
}