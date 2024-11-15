package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

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
public class StraightTest2 extends LinearOpMode {
    public static double DISTANCE = 40; // in
    public SampleMecanumDrive drive;
    public TwoWheelTrackingLocalizer track;
    public double currentMaxVel;
    private MinVelocityConstraint velocityConstraint;


    public static double MAX_VEL = 1200; // max velocity in inches per second
    public static double MIN_VEL = 0;
    public static double MAX_ACCEL = 1200;

    public void setDynamicVelocityConstraint(double maxVel) {
        this.velocityConstraint = new MinVelocityConstraint(Arrays.asList(
                new MinVelocityConstraint(Arrays.asList(
                        getVelocityConstraint(maxVel, MAX_ACCEL, TRACK_WIDTH)
                ))
        ));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        track = new TwoWheelTrackingLocalizer(hardwareMap, drive); // Adjust constructor parameters as needed

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        while (opModeIsActive() && !isStopRequested()) {
            // Get the current pose and the target pose (from trajectory)
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d targetPose = trajectory.end();

            // Calculate the remaining distance to the target
            double remainingDistance = calculateRemainingDistance(currentPose, targetPose);

            // Adjust max velocity based on remaining distance
            double adjustedMaxVel = Math.max(MIN_VEL, (remainingDistance / DISTANCE) * MAX_VEL);

            // Dynamically update the velocity constraint
            setDynamicVelocityConstraint(adjustedMaxVel);

            drive.update(); // Perform the robot's motion based on the new velocity constraints

            telemetry.addData("Remaining Distance", remainingDistance);
            telemetry.addData("Adjusted Max Velocity", adjustedMaxVel);
            telemetry.update();
        }
    }

    public double calculateRemainingDistance(Pose2d currentPose, Pose2d targetPose) {
        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY); // Euclidean distance formula
    }
}