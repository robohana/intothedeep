package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "ParkBlue_A3", preselectTeleOp = "CompTeleOp")
public class ParkBlue_A3 extends LinearOpMode {
    private SampleMecanumDrive drive;
    public static double RUNTIME = 20;

    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-29, 50, -1.5707963268);
        drive.setPoseEstimate(startPose);
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive() && timer.seconds() < RUNTIME) {
            telemetry.addData("Time Elapsed", timer.seconds());
            telemetry.addData("Waiting for", RUNTIME - timer.seconds());
            telemetry.update();
        }

        // After 20 seconds, perform the next actions
        if (opModeIsActive()) {
            telemetry.addData("Action", "Starting movement");
            telemetry.update();

            drive.followTrajectory(
                    drive.trajectoryBuilder(startPose)
                            .strafeRight(48) // Move forward 24 inches
                            .build()
            );
        }
    }
}

