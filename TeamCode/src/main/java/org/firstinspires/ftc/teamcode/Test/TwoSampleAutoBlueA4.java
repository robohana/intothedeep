package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ClawController;
import org.firstinspires.ftc.teamcode.Constants.hiJointPIDController;
import org.firstinspires.ftc.teamcode.Constants.vsPIDController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "2SP_AutoBlue_A4", preselectTeleOp = "CompTeleOp")
public class TwoSampleAutoBlueA4 extends LinearOpMode {
    public static double d_DISTANCE1 = 22; // in
    public static double d_DISTANCE2 = 50;
    public static double d_DISTANCE3 = 40;
    public static double d_DISTANCE4 = 8;
    public static double d_DISTANCE5 = 45;

    public static double s_DISTANCE1 = 20;
    public static double s_DISTANCE2 = 8;
    public static double s_DISTANCE3 = 50;

    public static double ANGLE1 = 45; // deg
    public static double ANGLE2 = 132; // deg

    private DcMotor hiJoint;
    public DcMotor leftviperSlide;
    public DcMotor rightviperSlide;
    public CRServo claw;

    public static double ARM_P = 0.005, ARM_I = 0, ARM_D = 0.0009, ARM_F = 0.1;
    private static final double ARM_TICKS_PER_DEGREE = 5281.1 / 180.0;
    private static final int ARM_TARGET = -500;
    private static final int ARM_TARGET2 = 0;

    public static double VS_P = 0.01, VS_I = 0, VS_D = 0.0001, VS_F = 0.1;
    private static final double VS_TICKS_PER_DEGREE = 537.7 / 180.0;
    private static final int VS_TARGET1 = 3500;
    private static final int VS_TARGET2 = 3900;
    private static final int VS_TARGET3 = 0;

    private SampleMecanumDrive drive;
    public org.firstinspires.ftc.teamcode.Constants.hiJointPIDController hiJointPIDController;
    public org.firstinspires.ftc.teamcode.Constants.vsPIDController vsPIDController;
    public ClawController clawController;

    public void moveJointToTarget(
            int targetPosition,
            long timeoutMillis,
            double tolerance
    ) {
        hiJointPIDController.setTarget(targetPosition); // Set the PID target
        long startTime = System.currentTimeMillis(); // Record start time

        // Loop until the target is reached or timeout expires
        while (opModeIsActive() &&
                Math.abs(hiJoint.getCurrentPosition() - hiJointPIDController.getTarget()) > tolerance &&
                (System.currentTimeMillis() - startTime) < timeoutMillis) {

            hiJointPIDController.update(); // Update PID

            // Telemetry for debugging
            telemetry.addData("hiJoint Position", hiJoint.getCurrentPosition());
            telemetry.addData("hiJoint Target", hiJointPIDController.getTarget());
            telemetry.addData("Elapsed Time", (System.currentTimeMillis() - startTime) / 1000.0);
            telemetry.update();
        }

        hiJoint.setPower(0); // Stop the joint
    }
    public void moveViperSlideToTarget(
            int targetPosition,
            double tolerance
    ) {
        vsPIDController.setTarget(targetPosition); // Set the PID target
        long startTime = System.currentTimeMillis(); // Record start time

        // Loop until the target is reached or timeout expires
        while (opModeIsActive() &&
                Math.abs(leftviperSlide.getCurrentPosition() - vsPIDController.getTarget()) > tolerance) {

            vsPIDController.update(); // Update PID

            // Telemetry for debugging
            telemetry.addData("LeftVS Position", leftviperSlide.getCurrentPosition());
            telemetry.addData("VS Target", vsPIDController.getTarget());
            telemetry.update();
        }

        // Stop both viper slides
        leftviperSlide.setPower(0);
        rightviperSlide.setPower(0);
    }

    public void runOpMode() throws InterruptedException {
        // makes it so that we can edit variable in dashboard instead of having to replug in every few seconds - LC 12/9
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        double voltage = getBatteryVoltage();
        double cpuUtilization = getCpuUtilization();

        //arm_controller = new PIDController(arm_p, arm_i, arm_d);

        claw = hardwareMap.get(CRServo.class, "claw");

        hiJoint = hardwareMap.get(DcMotor.class, "hiJoint");
        leftviperSlide = hardwareMap.get(DcMotor.class, "leftviperSlide");
        rightviperSlide = hardwareMap.get(DcMotor.class, "rightviperSlide");

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
        Pose2d startPose = new Pose2d(-1, 50, -1.5707963268);
        drive.setPoseEstimate(startPose);

        clawController = new ClawController(claw);

        long startTime = System.currentTimeMillis(); // Record the starting time


        hiJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hiJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftviperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftviperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
         * This trajectory sequence runs right now from the line up position to then push 2 of
         * the red samples into the observation zone for the human player to turn them in specimens. - LC 12/9
         *  Added in the vertical viper slides so that we can score a specimen that we preload
         *  and then be able to score more than one specimen during auto. - LC 12/10
         */
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .forward(22) // 22
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .forward(d_DISTANCE4) // 8
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .back(d_DISTANCE4) // 8
                .build();

        /*TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .turn(Math.toRadians(-ANGLE1)) // 45 deg
                .strafeRight(s_DISTANCE1) // 20
                .forward(d_DISTANCE3)// 40 - might change this one because it goes really far forward
                .turn(Math.toRadians(-ANGLE2)) // 135 deg
                .forward(d_DISTANCE3) // 40
                .back(8) // 50
                .strafeLeft(10) // 5
                .forward(10) // 10
                .build();*/

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory3.end())
                .turn(Math.toRadians(-180)) // 45 deg
                .strafeLeft(52) // 20
                .forward(25)// 40 - might change this one because it goes really far forward
                .build();

        TrajectorySequence trajectory5 = drive.trajectorySequenceBuilder(trajectory4.end())
                .back(10)
                .turn(Math.toRadians(-180)) // 90 deg
                .forward(18)// no clue yet what this distance has to be
                .strafeLeft(60) // 60 - go to the submursible again
                .build();

        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .forward(8) // 8
                .build();

        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end())
                .back(8) // 8
                .build();

        TrajectorySequence trajectory8 = drive.trajectorySequenceBuilder(trajectory7.end())
                .strafeRight(60) // 60
                .back(12) // 12
                .build();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        if (isStopRequested()) return;

        /*
         * Runs sequence, runs the joint angles up slightly to be out of the way of the viper
         * slides then it runs through the trajectory seq made above - LC 12/9
         */
        while (opModeIsActive()) {

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13
            moveJointToTarget(-600, 15000, 50);


            // runs us forward to the chambers in order to have the specimens, f: 22 - LC 12/10
            drive.followTrajectory(trajectory1);

            // puts the vs up high enough so we are in position to hang a sample once we move slightly forward - LC 12/10
            // Move viper slides to position 3500 with a tolerance of 50 - LC 12/13
            moveViperSlideToTarget(3000, 50);


            //gets us closer to the bar once our vs are up so that we can hang the specimen, f:8 - LC 12/10
            drive.followTrajectory(trajectory2);

            //open claw to release specimen - LC 12/13
            clawController.open();
            sleep(1000);

            // reverse so that we can lower the vs and be clear of the bars, b:8 - LC 12/10
            drive.followTrajectory(trajectory3);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13
            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(500);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            //run trajectory sequence where we move samples to the observation zone for the human player - LC 12/10
            drive.followTrajectorySequence(trajectory4);

            clawController.close();
            sleep(1000);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13
            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, just to get off the wall - LC 12/13
            moveViperSlideToTarget(1000, 50);

            drive.followTrajectorySequence(trajectory5);

            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, up to the chamber - LC 12/13
            moveViperSlideToTarget(3500, 50);

            drive.followTrajectory(trajectory6);

            //open claw to release specimen - LC 12/13
            clawController.open();
            sleep(1000);

            drive.followTrajectory(trajectory7);

            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(700);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            drive.followTrajectorySequence(trajectory8);


            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.addData("LeftVS Position", leftviperSlide.getCurrentPosition());
            telemetry.addData("VS Target", vsPIDController.getTarget());
            telemetry.addData("Battery Voltage", voltage);
            telemetry.addData("CPU Utilization", String.format("%.2f%%", cpuUtilization * 100));
            telemetry.update();

            while (!isStopRequested()) idle();
        }
    }
    public double getBatteryVoltage() {
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double currentVoltage = sensor.getVoltage();
            if (currentVoltage > 0) {
                voltage = Math.min(voltage, currentVoltage);
            }
        }

        return voltage;
    }
    public double getCpuUtilization() {
        Runtime runtime = Runtime.getRuntime();
        double usedMemory = runtime.totalMemory() - runtime.freeMemory();
        double maxMemory = runtime.maxMemory();
        return usedMemory / maxMemory;
    }
}



