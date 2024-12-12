package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DriveController {

    // Declare the drive motors
    private final DcMotor frontleftDrive;
    private final DcMotor frontrightDrive;
    private final DcMotor backleftDrive;
    private final DcMotor backrightDrive;
    private Telemetry telemetry; // Add telemetry reference


    // Constructor to initialize the drive motors
    public DriveController(DcMotor frontLeftDrive, DcMotor frontRightDrive, DcMotor backLeftDrive, DcMotor backRightDrive) {
        this.frontleftDrive = frontLeftDrive;
        this.frontrightDrive = frontRightDrive;
        this.backleftDrive = backLeftDrive;
        this.backrightDrive = backRightDrive;
    }

    /**
     * Handles the drivetrain control based on the gamepad inputs. - LC 12/10
     *
     * @param gamepad The gamepad used for driving controls. - LC 12/10
     */
    public void drive(Gamepad gamepad) {
        // Retrieve gamepad inputs - LC 12/10
        double axial = -gamepad.left_stick_y;  // Forward/backward
        double lateral = gamepad.left_stick_x;  // Strafing
        double yaw = gamepad.right_stick_x;    // Turning

        // Calculate power for each wheel - LC 12/10
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the power values to ensure they are within the range [-1.0, 1.0] - LC 12/10
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Apply scaling factor based on the right trigger - LC 12/10
        double scalingFactor = gamepad.right_trigger > 0.5 ? 0.3 : 0.8;
        leftFrontPower *= scalingFactor;
        rightFrontPower *= scalingFactor;
        leftBackPower *= scalingFactor;
        rightBackPower *= scalingFactor;

        // Send the calculated power to the motors - LC 12/10
        frontleftDrive.setPower(leftFrontPower);
        frontrightDrive.setPower(rightFrontPower);
        backleftDrive.setPower(leftBackPower);
        backrightDrive.setPower(rightBackPower);

        //telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontleftDrive, frontrightDrive);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backleftDrive, backrightDrive);
        //telemetry.update();

    }
}