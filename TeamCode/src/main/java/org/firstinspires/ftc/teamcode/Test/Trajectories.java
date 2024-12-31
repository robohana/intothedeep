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


    // Modify the initialize method to accept a Pose2d parameter
    public static void initialize(SampleMecanumDrive drive, Pose2d startPose) {
        trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(22) // 22
                .build();

        trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(8) // 8
                .build();

        trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .back(8) // 8
                .build();

        trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .turn(Math.toRadians(-180)) // 45 deg
                .strafeLeft(52) // 20
                .forward(25)// 40 - might change this one because it goes really far forward - LC 12/18
                .build();

        trajectory9 = drive.trajectoryBuilder(trajectory4.end())
                .back(10)
                .build();

        /*trajectory5 = drive.trajectorySequenceBuilder(trajectory9.end())
                .turn(Math.toRadians(-180)) // 90 deg
                .forward(18)// 18
                .strafeLeft(60) // 60 - go to the submersible again - LC 12/18
                .build();

        trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .forward(8) // 8
                .build();

        trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .back(8) // 8
                .build();

        trajectory8 = drive.trajectorySequenceBuilder(trajectory7.end())
                .strafeRight(60) // 60
                .back(12) // 12
                .build();*/

        trajSeq = drive.trajectorySequenceBuilder(trajectory9.end())
                .strafeRight(10) // 20
                .back(30)// 40
                .strafeLeft(7) // 40
                .forward(45)
                .back(50) // 50
                .strafeLeft(7) // 10
                .forward(45) // 45
                .build();
    }
}

