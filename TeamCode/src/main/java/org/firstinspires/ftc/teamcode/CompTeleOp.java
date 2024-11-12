package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="CompTeleOp")
public class CompTeleOp extends LinearOpMode {
    //decale all public guys - LC
    public DcMotor backleftDrive;
    public DcMotor backrightDrive;
    public DcMotor frontleftDrive;
    public DcMotor frontrightDrive;

    public DcMotor leftviperSlide;
    public DcMotor rightviperSlide;
    public CRServo claw;
    double vSPower;
    //double clawPower;

    double max;

    @Override
    public void runOpMode() throws InterruptedException {

        //handware map for Drivetrain - LC
        backleftDrive = hardwareMap.get(DcMotor.class, "BL");
        backrightDrive = hardwareMap.get(DcMotor.class, "BR");
        frontleftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontrightDrive = hardwareMap.get(DcMotor.class, "FR");

        //hardware map for butter gripper tower of terror - LC
        leftviperSlide = hardwareMap.get(DcMotor.class, "leftviperSlide");
        rightviperSlide = hardwareMap.get(DcMotor.class, "rightviperSlide");
        claw = hardwareMap.get(CRServo.class, "claw");


        //this sets the direction the motor will spin for the drive wheels - LC
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);

        //sets the directions of the two parallel viper slides in opposite directions so they will move up and down in unison. - LC
        leftviperSlide.setDirection(DcMotor.Direction.FORWARD);
        rightviperSlide.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(opModeIsActive()) {
                /* set's it so that the butter gripper will be contantly closed when you are not
                holding the button. it is a continuous servo just is really set up like a regular servo
                with how zero is the open position and 1 is the closed butter position - LC  */

                if (gamepad2.right_bumper) {         // go max open position - LC
                    claw.setPower(-1);
                } else {                          // go to closed position
                    claw.setPower(1);
                }
            }



            //sets the power that goes to the viper slides to be relyant on gamepad 2 (opertator) left stick when you move it in the y direction - LC
            double vSPower = -gamepad2.left_stick_y;
            leftviperSlide.setPower(vSPower);
            rightviperSlide.setPower(vSPower);

            //Controlls for gamepad 1 (driver) - LC
            double axial   = -gamepad1.left_stick_y;  // Forward
            double lateral =  gamepad1.left_stick_x;  //strafe
            double yaw     =  gamepad1.right_stick_x; //spin turn

            // Combine the joystick requests for each axis-motion to determine each wheel's power. - LC
            // Set up a variable for each drive wheel to save the power level for telemetry. - LC
            double leftfrontPower  = axial + lateral + yaw;
            double rightfrontPower = axial + lateral - yaw;
            double leftbackPower   = axial - lateral + yaw;
            double rightbackPower  = axial - lateral - yaw;

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
            telemetry.addData("ViperSlide Power","%4.2f", vSPower);
            //telemetry.addData("Claw Power","%4.2f", clawPower);
            telemetry.update();

        }
    }

}






