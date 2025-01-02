package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Trajectories {
    public static Trajectory trajectory1;
    public static Trajectory trajectory2;
    public static Trajectory trajectory3;
    public static Trajectory trajectory9;
    public static Trajectory trajectory6;
    public static Trajectory trajectory7;

    public static TrajectorySequence trajectory4;
    public static TrajectorySequence trajectory5;
    public static TrajectorySequence trajectory8;
    public static TrajectorySequence trajSeq;

    /*
     * Quick to just get this out of the way, trajectory acronyms. F = Forward, B = Back, SL = Strafe Left, SR = Strafe Right, T = Turn, S = Spline, anything else = too late at night coding - LC 12/30/24
     */

    // Modify the initialize method to accept a Pose2d parameter - LC 12/30/24
    public static void initialize(SampleMecanumDrive drive, Pose2d startPose) {
        // F = 22 - LC 12/30/24
        trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(22) // 22
                .build();

        // F = 8 - LC 12/30/24
        trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(8) // 8
                .build();

        // B = 8 - LC 12/30/24
        trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .back(8) // 8
                .build();

        // T = -180, SL = 52, F = 25 - LC 12/30/24
        trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .turn(Math.toRadians(-180)) // 180 deg
                .strafeLeft(52) // 52
                .forward(25) // 25
                .build();

        // B = 10 - LC 12/30/24
        trajectory9 = drive.trajectoryBuilder(trajectory4.end())
                .back(10) // 10
                .build();

        // T = -180, F = 18, SL = 60 - LC 12/30/24
        trajectory5 = drive.trajectorySequenceBuilder(trajectory9.end())
                .turn(Math.toRadians(-180)) // 180 deg
                .forward(18) // 18
                .strafeLeft(60) // 60 - go to the submersible again - LC 12/18
                .build();

        // F = 8 - LC 12/30/24
        trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .forward(8) // 8
                .build();

        // B = 8 - LC 12/30/24
        trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .back(8) // 8
                .build();

        // SR = 60, B = 12 - LC 12/30/24
        trajectory8 = drive.trajectorySequenceBuilder(trajectory7.end())
                .strafeRight(60) // 60
                .back(12) // 12
                .build();

        // SL = 10, B = 30, SL = 7, F = 45, B = 50, SL = 7, F = 45 - LC 12/30/24
        trajSeq = drive.trajectorySequenceBuilder(trajectory9.end())
                .strafeRight(10) // 10
                .back(30)// 30
                .strafeLeft(7) // 7
                .forward(45) // 45
                .back(50) // 50
                .strafeLeft(7) // 7
                .forward(45) // 45
                .build();
    }
}

