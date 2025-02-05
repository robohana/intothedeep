package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name = "Odo Pod Test",group = "test")
public class odopodresue extends LinearOpMode {

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;

    public void runOpMode() throws InterruptedException {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "hiExtend"));

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("leftEncoder value", leftEncoder.getCurrentPosition());
            telemetry.addData("rEncoder value", rightEncoder.getCurrentPosition());
            telemetry.addData("fEncoder value", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
