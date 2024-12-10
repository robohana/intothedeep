package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Test.hiJointPIDController;
import org.firstinspires.ftc.teamcode.Test.vsPIDController;


@Config
@Autonomous(name = "AutoRedOdo")//, preselectTeleOp = "CompTeleOp")
public class AutoRedOdo extends LinearOpMode {
    public static double d_DISTANCE1 = 24; // in
    public static double d_DISTANCE2 = 20;
    public static double d_DISTANCE3 = 50;
    public static double d_DISTANCE4 = 10;

    public static double s_DISTANCE1 = 56;
    public static double s_DISTANCE2 = 5;
    public static double s_DISTANCE3 = 50;

    public static double ANGLE1 = 180; // deg
    public static double ANGLE2 = 90; // deg

    public static double RUNTIME = 20;

    private DcMotor hiJoint;
    public DcMotor leftviperSlide;
    public DcMotor rightviperSlide;

    public static double ARM_P = 0.005, ARM_I = 0, ARM_D = 0.0009, ARM_F = 0.1;
    private static final double ARM_TICKS_PER_DEGREE = 5281.1 / 180.0;
    private static final int ARM_TARGET = -300;

    public static double VS_P = 0, VS_I = 0, VS_D = 0, VS_F = 0;
    private static final double VS_TICKS_PER_DEGREE = 5281.1 / 180.0;
    private static final int VS_TARGET = -300;

    private SampleMecanumDrive drive;
    public hiJointPIDController hiJointPIDController;
    public vsPIDController vsPIDController;



    public void score(){

    }

    public void runOpMode() throws InterruptedException {
        // makes it so that we can edit variable in dashboard instead of having to replug in every few seconds - LC 12/9
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        //arm_controller = new PIDController(arm_p, arm_i, arm_d);

        hiJoint = hardwareMap.get(DcMotor.class, "hiJoint");

        hiJointPIDController = new hiJointPIDController(hiJoint, ARM_P, ARM_I, ARM_D, ARM_F, ARM_TICKS_PER_DEGREE);
        vsPIDController = new vsPIDController(leftviperSlide, rightviperSlide, VS_P, VS_I, VS_D, VS_F, VS_TICKS_PER_DEGREE);

        /*
        * This start pose is specifically for the orientation where the robot is facing the submersible
        * on the right side. It is on the tile that is to the left of the field x axis. Then the right
        * set of wheels are right against the joint of the two ties. They are not on the join of the two
        * tiles but right next to it. - LC 12/9
        * X and Y are in inches - LC 12/8
        * Heading is in Radians - LC 12/8
        */
        Pose2d startPose = new Pose2d(-1, -50, 1.5707963268);
        drive.setPoseEstimate(startPose);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        /*
         * This trajectory sequence runs right now from the line up position to then push 2 of
         * the red samples into the observation zone for the human player to turn them in specimens. - LC 12/9
         * TODO: Add in the vertical viper slides so that we can score a specimen that we preload
         *  and then be able to score more than one specimen during auto. - LC 12/9
         */
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(d_DISTANCE1)
                .build();
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(trajectory1.end())
                //.forward(d_DISTANCE1)
                .strafeRight(s_DISTANCE1)
                .forward(d_DISTANCE2)
                .turn(Math.toRadians(-ANGLE1))
                .forward(d_DISTANCE3)
                .back(d_DISTANCE3)
                .strafeLeft(s_DISTANCE2)
                .forward(d_DISTANCE3)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            /*
            * Runs sequence, runs so the joint angles up slightly to be out of the way of the viper
            * slides then it runs through the trajectory seq made above - LC 12/9
            */
            //hiJoint_PID();
            hiJointPIDController.setTarget(ARM_TARGET);
            hiJointPIDController.update();
            drive.followTrajectory(trajectory1);
            /*vsPIDController.setTarget(VS_TARGET);
            vsPIDController.update();*/
            drive.followTrajectorySequence(trajSeq);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();

            while (!isStopRequested()) idle();
        }
    }
}
