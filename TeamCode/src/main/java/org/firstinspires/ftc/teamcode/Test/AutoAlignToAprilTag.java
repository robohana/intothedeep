package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@Autonomous(group = "drive")
@Disabled
public class AutoAlignToAprilTag extends OpMode {

    // Limelight and robot variables
    double targetDistance = 12.0; // Desired distance from the target (in inches)
    double driveSpeed = 0.1; // Speed factor for driving toward the target
    double turnSpeed = 0.1; // Speed factor for rotating toward the target
    double distanceThreshold = 1.0; // Threshold for how close to the target the robot should get (in inches)
    double angleThreshold = 2.0; // Threshold for how accurate the robot's heading should be (in degrees)

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    private Limelight3A limelight;
    String distance;
    public double drive = 0;
    public double strafe = 0;
    public double turn = 0;


    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

    }

    @Override
    public void loop() {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        // Get the current status from Limelight
        LLResult result = limelight.getLatestResult();
        List<FiducialResult> fiducials = result.getFiducialResults();
        //boolean targetFound = result.isValid();

        Pose3D botpose = result.getBotpose();
        if (botpose != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
        }

        /*List<FiducialResult> fiducials = result.getFiducialResults();
        for (FiducialResult fr : fiducials) {
            int id = fr.getFiducialId(); // The ID number of the fiducial
            double x = fr.getTargetXDegrees(); // Where it is (left-right)
            double y = fr.getTargetYDegrees(); // Where it is (up-down)
            Pose3D StrafeDistance_3D = fr.getRobotPoseTargetSpace();
            telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
        }
        if (targetFound) {
            // Get AprilTag information
            double tx = result.getTx(); // Horizontal offset in degrees
            double ty = result.getTy(); // Vertical offset in degrees
            Pose3D botpose = result.getBotpose();
            double distance = result.getTargetingLatency(); // Distance to target in inches

            // Output current targeting data for debugging
            telemetry.addData("Target Found", "Yes");
            telemetry.addData("Horizontal Offset (Tx)", tx);
            telemetry.addData("Vertical Offset (Ty)", ty);
            telemetry.addData("Distance to Target", distance);

            // Calculate errors for driving toward and aligning with the target
            double distanceError = distance - targetDistance; // The difference in distance
            double angleError = tx; // The horizontal offset (angle error)

            // Decide if we're close enough to stop adjusting
            if (Math.abs(distanceError) < distanceThreshold && Math.abs(angleError) < angleThreshold) {
                // If the robot is close enough to the target, stop moving
                stopRobot();
            } else {
                // Drive toward the target
                double drive = Math.signum(distanceError) * driveSpeed * Math.abs(distanceError);
                double turn = Math.signum(angleError) * turnSpeed * Math.abs(angleError);

                // Clamp values to avoid exceeding maximum robot speeds
                drive = Range.clip(drive, -1.0, 1.0);
                turn = Range.clip(turn, -1.0, 1.0);

                // Move robot based on calculated drive and turn values
                moveRobot(drive, 0, turn);
            }
        } else {
            // If no target is found, stop the robot
            telemetry.addData("Target Found", "No Target Found");
            stopRobot();
        }

        telemetry.update(); // Update telemetry display
*/

        double drive = 0.0;
        double strafe = 0.0;
        double turn = 0.0;
        boolean targetFound = false;

        for (FiducialResult fr : fiducials) {
            Pose3D robotPose = fr.getRobotPoseTargetSpace(); // Robot pose relative to fiducial
            int id = fr.getFiducialId(); // ID of the fiducial
            double tx = fr.getTargetXDegrees(); // Horizontal offset
            double ty = fr.getTargetYDegrees(); // Vertical offset

            if (robotPose != null) {
                targetFound = true; // We have a valid fiducial
                double distance = robotPose.getPosition().x; // Direct access to the x field (if available)
                double angleError = tx; // Angular misalignment

                telemetry.addData("Fiducial ID", id);
                telemetry.addData("Distance to Target (meters)", "%.2f", distance);
                telemetry.addData("Horizontal Offset (degrees)", "%.2f", tx);
                telemetry.addData("Vertical Offset (degrees)", "%.2f", ty);

                // Autonomous control logic
                // Adjust drive speed based on distance to target
                if (distance > 0.2) { // Threshold for stopping near the target
                    drive = Math.signum(distance) * -0.5; // Drive towards the target
                } else if (distance < -0.2) { // Threshold for stopping near the target
                    drive = Math.signum(distance) * 0.5; // Drive towards the target
                }

                // Adjust turning based on horizontal alignment
                if (angleError > 2.0) { // Threshold for alignment
                    turn = Math.signum(angleError) * 0.3; // Turn to align
                }

                // Adjust strafing based on horizontal position
                if (tx > 0.1) { // Threshold for lateral adjustment
                    strafe = Math.signum(tx) * 0.4; // Strafe to align
                }
            }
        }

        if (!targetFound) {
            telemetry.addData("Target Found", "No");
            // If no target is found, the robot can stop or search
            drive = 0;
            strafe = 0;
            turn = 0;
        }

        // Clip speeds to ensure they stay within limits
        /*drive = Range.clip(drive, -1.0, 1.0);
        strafe = Range.clip(strafe, -1.0, 1.0);
        turn = Range.clip(turn, -1.0, 1.0);*/

        // Move the robot based on calculated values
        moveRobot(drive, strafe, turn);

        telemetry.update();
    }

    // Method to control robot movement
    public void moveRobot(double drive, double strafe, double turn) {
        double leftFrontPower = drive - strafe - turn;
        double rightFrontPower = drive + strafe + turn;
        double leftBackPower = drive + strafe - turn;
        double rightBackPower = drive - strafe + turn;

        // Normalize the power values
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        double scalingFactor = 0.3; //Adjust this value between 0 and 1 to reduce power, 1 is normal power, 0 is no power, don't go over 1 or under 0 - LC
        //apply scaling factor - LC
        leftFrontPower  *= scalingFactor;
        rightFrontPower *= scalingFactor;
        leftBackPower   *= scalingFactor;
        rightBackPower  *= scalingFactor;

        // Set motor power values
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // Method to stop the robot
    public void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}

