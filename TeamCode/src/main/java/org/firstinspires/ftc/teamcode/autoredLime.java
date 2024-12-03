package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

import java.util.Arrays;

@Config
@Autonomous(group = "drive")
public class autoredLime extends LinearOpMode {
    public static double DISTANCE1 = 24; // in
    public static double DISTANCE2 = 60;
    public SampleMecanumDrive drive;
    public TwoWheelTrackingLocalizer track;

    public static double MAX_VEL = 1000; // max velocity in inches per second
    public static double MAX_ACCEL = 1000;
    public static double ANGLE = 90; // deg

    //@Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        track = new TwoWheelTrackingLocalizer(hardwareMap, drive); // Adjust constructor parameters as needed

        /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE1,
                        new MinVelocityConstraint(Arrays.asList(
                                new MinVelocityConstraint(Arrays.asList(
                                        drive.getVelocityConstraint(MAX_VEL, MAX_ACCEL, TRACK_WIDTH)
                                ))
                        )),
                        new ProfileAccelerationConstraint(MAX_ACCEL)
                )
                .strafeRight(DISTANCE2)
                .build();*/

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE1)
                .build();


        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeRight(DISTANCE2)
                //.forward(DISTANCE2)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //drive.followTrajectory(trajectory);
        drive.followTrajectory(trajectory1);
        sleep(500);
        //drive.turn(Math.toRadians(ANGLE));
        //sleep(500);
        drive.followTrajectory(trajectory2);


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        //telemetry.addData("Parallel Encoder Position", track.parallelEncoder.getCurrentPosition());
        //telemetry.addData("Perpendicular Encoder Position", track. perpendicularEncoder.getCurrentPosition());
        //telemetry.addData("Wheel Positions (inches)", track.getWheelPositions());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
