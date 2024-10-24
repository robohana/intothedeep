package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drive.opmode.TestSampleMecanumDrive;

import java.util.Locale;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    TestSampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new TestSampleMecanumDrive(hardwareMap);

        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo"); // Ensure the name "odo" matches your configuration
        } catch (IllegalArgumentException e) {
            telemetry.addData("Error", "GoBildaPinpointDriver not found. Ensure it's correctly configured in the FTC app.");
            telemetry.update();
            while (!isStopRequested()) {
                // Wait for the user to stop the OpMode
            }
            return; // Exit the OpMode if odo is not found
        }

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        odo.resetPosAndIMU();
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )

            );

            drive.botUpdate();
            //drive.updateOdometry();
            odo.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));
            telemetry.addData("Position", data);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
