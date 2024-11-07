package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Omni")
public class Omni extends LinearOpMode {
    public DcMotor backleftDrive;
    public DcMotor backrightDrive;
    public DcMotor frontleftDrive;
    public DcMotor frontrightDrive;

    double max;

    public void runOpMode() throws InterruptedException {
        backleftDrive = hardwareMap.get(DcMotor .class, "BL");
        backrightDrive = hardwareMap.get(DcMotor.class, "BR");
        frontleftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontrightDrive = hardwareMap.get(DcMotor.class, "FR");

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            double axial   = -gamepad1.left_stick_y;  // Forward
            double lateral =  gamepad1.left_stick_x;  //strafe
            double yaw     =  gamepad1.right_stick_x; //spin turn

            // Combine the joystick requests for each axis-motion to determine each wheel's power. - LC
            // Set up a variable for each drive wheel to save the power level for telemetry. - LC
            double leftfrontPower  = axial - lateral + yaw;
            double rightfrontPower = axial - lateral - yaw;
            double leftbackPower   = axial + lateral + yaw;
            double rightbackPower  = axial + lateral - yaw;

            // Normalize the values - LC
            max = Math.max(Math.abs(leftfrontPower), Math.abs(rightfrontPower));
            max = Math.max(max, Math.abs(leftbackPower));
            max = Math.max(max, Math.abs(rightbackPower));

            if (max < 0.1) {
                leftfrontPower  /= max;
                rightfrontPower /= max;
                leftbackPower   /= max;
                rightbackPower  /= max;
            }

            //define a scaling factor to reduce power - LC
            double scalingFactor = 0.5; //Adjust this value between 0 and 1 to reduce power, 1 is normal power, 0 is no power, don't go over 1 or under 0 - LC

            //apply scaling factor - LC
            leftfrontPower  *= scalingFactor;
            rightfrontPower *= scalingFactor;
            leftbackPower   *= scalingFactor;
            rightbackPower  *= scalingFactor;

            //Send calculated power to wheels - LC
            frontleftDrive.setPower(leftfrontPower);
            frontrightDrive.setPower(rightfrontPower);
            backleftDrive.setPower(leftbackPower);
            backrightDrive.setPower(rightbackPower);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftfrontPower, rightfrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftbackPower, rightbackPower);
            telemetry.update();
        }
    }

}
