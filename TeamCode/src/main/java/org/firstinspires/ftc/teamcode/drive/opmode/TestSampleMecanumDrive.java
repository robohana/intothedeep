package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

@Config

public class TestSampleMecanumDrive extends MecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public TrajectorySequenceRunner trajectorySequenceRunner;
    public static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    public TrajectoryFollower follower;
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public List<DcMotorEx> motors = new ArrayList<>();
    public VoltageSensor batteryVoltageSensor;
    public List<Integer> lastEncPositions = new ArrayList<>();
    public List<Integer> lastEncVels = new ArrayList<>();
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public Pose2D currentPose;
    Pose2D lastPose;
    Pose2D startPose;
    Pose2D trajectory;
    //Pose2D startPose = odo.getPosition();  // Example of getting the FTC Pose2D
    //Pose2d convertedStartPose = convertPose2dToPose2D(startPose);  // Convert from Road Runner Pose2d

    private Pose2d convertPose2DToPose2d(Pose2D startPose) {
        // Extract the x, y, and heading values from the startPose
        double x = startPose.getX(DistanceUnit.INCH);   // Use your preferred DistanceUnit, like INCH or MM
        double y = startPose.getY(DistanceUnit.INCH);   // Use INCH, MM, etc., depending on your needs
        double heading = startPose.getHeading(AngleUnit.RADIANS);  // Road Runner uses RADIANS for angles

        // Return a new Pose2d object using the extracted values
        return new Pose2d(x, y, heading);
    }


    public TestSampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize the odometry driver with exception handling
        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            if (odo == null) {
                throw new NullPointerException("Odometry driver is null");
            }
            odo.setOffsets(-84.0, -168.0); // Tuned values
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        } catch (Exception e) {
            //telemetry.addData("Error", "Odometry Initialization Failed: " + e.getMessage());
            //telemetry.update();
            return; // Exit if initialization fails
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "frontleftDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "backleftDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "backrightDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontrightDrive");

        motors.add(leftFront);
        motors.add(leftRear);
        motors.add(rightRear);
        motors.add(rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Initialize trajectory runner
        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, new ArrayList<>(), new ArrayList<>()
        );

        currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2D startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    /*public void updateOdometry() {
        // Retrieve encoder values
        Pose2D newPose = odo.getPosition(); // Assuming this method returns a Pose2D

        // Update the currentPose with the new pose data
        currentPose = new Pose2D(
                DistanceUnit.INCH,
                newPose.getX(DistanceUnit.INCH),
                newPose.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                newPose.getHeading(AngleUnit.RADIANS)
        );
    }*/

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(odo.getPosition())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    private TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d start) {
    return null;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

   /* public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }*/

    public void botUpdate() {


        odo.update();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity()); // Use currentPose instead of getPoseEstimate()
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            botUpdate();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        //this.drivePower = drivePower;
        //Pose2d vel = drivePower;
        odo.update();

        Pose2D newPose = odo.getPosition();
        currentPose = new Pose2D(
                DistanceUnit.INCH,
                newPose.getX(DistanceUnit.INCH),
                newPose.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                newPose.getHeading(AngleUnit.RADIANS)
        );

        Pose2D vel = odo.getVelocity();

        odo.update();
    }
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        Pose2D pos = odo.getPosition(); // Get the current pose

        // Create a list of doubles to store X, Y, and heading values
        List<Double> wheelPositions = new ArrayList<>();

        // Add the X, Y, and heading values in their respective units
        wheelPositions.add(pos.getX(DistanceUnit.INCH));  // X in inches
        wheelPositions.add(pos.getY(DistanceUnit.INCH));  // Y in inches
        wheelPositions.add(pos.getHeading(AngleUnit.RADIANS));  // Heading in radians

        // Optionally, log the formatted string for debugging/telemetry
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Position", data);  // For telemetry display

        // Return the list of positions
        return wheelPositions;
    }

        public List<Double> getWheelVelocities() {
            Pose2D vel = odo.getVelocity(); // Get the current velocity

            // Create a list of doubles to store X, Y, and heading velocities
            List<Double> wheelVelocities = new ArrayList<>();

            // Add the X, Y, and heading velocities in their respective units
            wheelVelocities.add(vel.getX(DistanceUnit.INCH));  // X velocity in inches/sec
            wheelVelocities.add(vel.getY(DistanceUnit.INCH));  // Y velocity in inches/sec
            wheelVelocities.add(vel.getHeading(AngleUnit.RADIANS));  // Heading velocity in radians/sec

            // Optionally, log the formatted string for debugging/telemetry
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                    vel.getX(DistanceUnit.INCH),
                    vel.getY(DistanceUnit.INCH),
                    vel.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Velocity", velocity);  // For telemetry display

            // Return the list of velocities
            return wheelVelocities;
        }

        public void setMotorPowers(double v, double v1, double v2, double v3) {
            // Set power to the left front motor
            leftFront.setPower(v);
            // Set power to the left rear motor
            leftRear.setPower(v1);
            // Set power to the right rear motor
            rightRear.setPower(v2);
            // Set power to the right front motor
            rightFront.setPower(v3);
        }

    //@Override
    public List<Double> getHeading() {
        return Collections.singletonList(odo.getHeading(AngleUnit.RADIANS));
    }


    /*public Double getExternalHeadingVelocity() {
        return (double) odo.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }*/

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

        protected double getRawExternalHeading() {
        return 0; // Implement your heading logic here
    }
}
