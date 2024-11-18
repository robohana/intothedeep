package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="PIDvs")
public class PIDvs extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degee = 5281.1 / 180.0;

    public DcMotor leftviperSlide;
    public DcMotor rightviperSlide;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftviperSlide = hardwareMap.get(DcMotor.class, "leftviperSlide");
        rightviperSlide = hardwareMap.get(DcMotor.class, "rightviperSlide");

    }
    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int vsPos = leftviperSlide.getCurrentPosition();
        double pid = controller.calculate(vsPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degee)) * f;

        double power = pid + ff;

        leftviperSlide.setPower(power);
        rightviperSlide.setPower(power);

        telemetry.addData("pos", vsPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
