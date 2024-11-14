package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

import java.util.Arrays;
import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static double DISTANCE = 40; // in
    public SampleMecanumDrive drive;
    public TwoWheelTrackingLocalizer track;

    public static double MAX_VEL = 1000; // max velocity in inches per second
    public static double MAX_ACCEL = 1000;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        track = new TwoWheelTrackingLocalizer(hardwareMap, drive); // Adjust constructor parameters as needed

        // Debug print the distance and the unit being used
        telemetry.addData("DISTANCE_UNIT", DISTANCE_UNIT.toString());
        telemetry.addData("DISTANCE", DISTANCE);
        telemetry.update();

        //double distanceInDesiredUnit = convertToDistanceUnit(DISTANCE);


        /*Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();*/

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE,
                        new MinVelocityConstraint(Arrays.asList(
                                new MinVelocityConstraint(Arrays.asList(
                                        drive.getVelocityConstraint(MAX_VEL, MAX_ACCEL, TRACK_WIDTH)
                                ))
                        )),
                        new ProfileAccelerationConstraint(MAX_ACCEL)
                )
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("Parallel Encoder Position", track.parallelEncoder.getCurrentPosition());
        telemetry.addData("Perpendicular Encoder Position", track. perpendicularEncoder.getCurrentPosition());
        telemetry.addData("Wheel Positions (inches)", track.getWheelPositions());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }

}
