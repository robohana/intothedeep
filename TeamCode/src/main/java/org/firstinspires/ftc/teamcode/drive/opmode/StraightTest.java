package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static DistanceUnit DISTANCE_UNIT = DistanceUnit.INCH;
    public static double DISTANCE = 60; // in
    public SampleMecanumDrive drive;
    public TwoWheelTrackingLocalizer track;

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


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("wheel Pos", track.getWheelPositions());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
    /**
     * Converts the distance from the specified unit to the desired unit (INCH or CM).
     * @param distance The distance to convert
     * @return The converted distance in the desired unit.
     */
    /*private double convertToDistanceUnit(double distance) {
        if (DISTANCE_UNIT == DistanceUnit.CM) {
            // Convert from inches to centimeters
            return distance * 2.54;
        }
        // Default is inches, so return the original value.
        return distance;
    }*/
}
