package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Omni")
public class Omni extends LinearOpMode {
    public DcMotor backleftDrive;
    public DcMotor backrightDrive;
    public DcMotor frontleftDrive;
    public DcMotor frontrightDrive;

    //public DcMotor VS;

    double max;

    public void runOpMode() throws InterruptedException {

        int zrev = 0;
        int srev = 50000;

        backleftDrive = hardwareMap.get(DcMotor .class, "BL");
        backrightDrive = hardwareMap.get(DcMotor.class, "BR");
        frontleftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontrightDrive = hardwareMap.get(DcMotor.class, "FR");
        //VS = hardwareMap.get(DcMotor.class, "VS");
        //DcMotor ENVS = hardwareMap.dcMotor.get("VS");

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*ENVS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ENVS.setTargetPosition(zrev);
        ENVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ENVS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        waitForStart();
        while (opModeIsActive()) {

            /*int VSposition = ENVS.getCurrentPosition();
            VSposition = clamp(VSposition, zrev, srev);

            double power = -gamepad2.left_stick_y;
            if ((VSposition <= zrev && power < 0) || (VSposition >= srev && power > 0)) {
                power = 0;  // Stop the motor if it reaches the limit in either direction - LC
            }
            VS.setPower(power);*/

            double axial   = -gamepad1.left_stick_y;  // Forward
            double lateral =  gamepad1.left_stick_x;  //strafe
            double yaw     =  gamepad1.right_stick_x; //spin turn

            // Combine the joystick requests for each axis-motion to determine each wheel's power. - LC
            // Set up a variable for each drive wheel to save the power level for telemetry. - LC
            double leftfrontPower  = axial + lateral + yaw;
            double rightfrontPower = axial - lateral - yaw;
            double leftbackPower   = axial - lateral + yaw;
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

            if (gamepad1.right_trigger > 0.5){
                double scalingFactor = 0.3; //Adjust this value between 0 and 1 to reduce power, 1 is normal power, 0 is no power, don't go over 1 or under 0 - LC
                //apply scaling factor - LC
                leftfrontPower  *= scalingFactor;
                rightfrontPower *= scalingFactor;
                leftbackPower   *= scalingFactor;
                rightbackPower  *= scalingFactor;
            } else {
                double scalingFactor = 0.7; //Adjust this value between 0 and 1 to reduce power, 1 is normal power, 0 is no power, don't go over 1 or under 0 - LC
                //apply scaling factor - LC
                leftfrontPower  *= scalingFactor;
                rightfrontPower *= scalingFactor;
                leftbackPower   *= scalingFactor;
                rightbackPower  *= scalingFactor;
            }
            //define a scaling factor to reduce power - LC
            /*double scalingFactor = 0.5; //Adjust this value between 0 and 1 to reduce power, 1 is normal power, 0 is no power, don't go over 1 or under 0 - LC

            //apply scaling factor - LC
            leftfrontPower  *= scalingFactor;
            rightfrontPower *= scalingFactor;
            leftbackPower   *= scalingFactor;
            rightbackPower  *= scalingFactor;*/

            //Send calculated power to wheels - LC
            frontleftDrive.setPower(leftfrontPower);
            frontrightDrive.setPower(rightfrontPower);
            backleftDrive.setPower(leftbackPower);
            backrightDrive.setPower(rightbackPower);

            /*if (gamepad2.a){
                ENVS.setTargetPosition(srev);
                ENVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ENVS.setPower(0.1);
            } else if (gamepad2.b){
                ENVS.setTargetPosition(zrev);
                ENVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ENVS.setPower(0.1);
            } else{
                ENVS.setPower(0);
            }*/


            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftfrontPower, rightfrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftbackPower, rightbackPower);
            //telemetry.addData("En VS Position", VSposition);
            telemetry.update();
        }
    }
    // Clamping function to keep encoder within the desired range - LC
    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(value, max));
    }

}
