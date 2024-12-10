package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Test.hiJointPIDController;
import org.firstinspires.ftc.teamcode.Test.vsPIDController;


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

    public DcMotor hiJoint;
    public DcMotor hiExtend;
    public CRServo intakeRoller;

    private boolean lockEnabled = false;

    public static double ARM_P = 0.005, ARM_I = 0, ARM_D = 0.0009, ARM_F = 0.1;
    private static final double ARM_TICKS_PER_DEGREE = 5281.1 / 180.0;
    public hiJointPIDController hiJointPIDController;

    public static double VS_P = 0, VS_I = 0, VS_D = 0, VS_F = 0;
    private static final double VS_TICKS_PER_DEGREE = 5281.1 / 180.0;
    private static final int VS_TARGET = -300;
    public vsPIDController vsPIDController;

    double max;

    @Override
    public void runOpMode() throws InterruptedException {

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //handware map for Drivetrain - LC
        backleftDrive = hardwareMap.get(DcMotor.class, "BL");
        backrightDrive = hardwareMap.get(DcMotor.class, "BR");
        frontleftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontrightDrive = hardwareMap.get(DcMotor.class, "FR");

        //hardware map for butter gripper tower of terror - LC
        leftviperSlide = hardwareMap.get(DcMotor.class, "leftviperSlide");
        rightviperSlide = hardwareMap.get(DcMotor.class, "rightviperSlide");
        claw = hardwareMap.get(CRServo.class, "claw");

        //hardware map for butter gripper tower of terror - LC
        hiJoint = hardwareMap.get(DcMotor.class, "hiJoint");
        hiExtend = hardwareMap.get(DcMotor.class, "hiExtend");
        intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");

        hiJointPIDController = new hiJointPIDController(hiJoint, ARM_P, ARM_I, ARM_D, ARM_F, ARM_TICKS_PER_DEGREE);
        vsPIDController = new vsPIDController(leftviperSlide, rightviperSlide, VS_P, VS_I, VS_D, VS_F, VS_TICKS_PER_DEGREE);


        //this sets the direction the motor will spin for the drive wheels - LC
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);

        //sets the directions of the two parallel viper slides in opposite directions so they will move up and down in unison. - LC
        leftviperSlide.setDirection(DcMotor.Direction.FORWARD);
        rightviperSlide.setDirection(DcMotor.Direction.REVERSE);

        //sets the directions of the two parallel viper slides in opposite directions so they will move up and down in unison. - LC
        hiJoint.setDirection(DcMotor.Direction.FORWARD);
        hiExtend.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.x && !lockEnabled) {
                lockEnabled = true;
            }

            // If locked, set all motors to zero and skip further processing
            if (lockEnabled) {
                hold();
                telemetry.addData("Status", "Locked");
                telemetry.update();
                continue;
            }

            //sets the power that goes to the viper slides to be relyant on gamepad 2 (opertator) left stick when you move it in the y direction - LC
            double vSPower = -gamepad2.left_stick_y;
            leftviperSlide.setPower(vSPower);
            rightviperSlide.setPower(vSPower);

            //sets the power that goes to the joint on the hanger and intake to be relyant on gamepad 2 (opertator) right stick when you move it in the y direction - LC
            double jointPower = -gamepad2.right_stick_y;
            /*if (-gamepad2.right_stick_y > 0.5){
                double scalingFactor = 0.5;
                jointPower  *= scalingFactor;
                hiJoint.setPower(jointPower);
            }*/ //if else (-gamepad2.right_stick_y < 0.5){

            //}
            hiJoint.setPower(jointPower);

            //Controlls for gamepad 1 (driver) - LC
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
                leftfrontPower *= scalingFactor;
                rightfrontPower *= scalingFactor;
                leftbackPower *= scalingFactor;
                rightbackPower *= scalingFactor;
            }

            //Send calculated power to wheels - LC
            frontleftDrive.setPower(leftfrontPower);
            frontrightDrive.setPower(rightfrontPower);
            backleftDrive.setPower(leftbackPower);
            backrightDrive.setPower(rightbackPower);

            /* set's it so that the butter gripper will be right all the way open, left all the way
            close or prob just chill. hopefully just chill. This is also with a CR Servo so 1 and
            -1 are max power to the system. you can do half power but DON'T SET THE POWER -1<=X<=1
            OR I WILL FIND YOU!!- LC  */
            if (gamepad2.right_bumper) {         // go max open position - LC
                claw.setPower(-1);
            } else if (gamepad2.left_bumper) {   // go to closed position - LC
                claw.setPower(1);
            } else {                             // go to no power - LC
                claw.setPower(0);
            }

            //set power and control of the servo for intake to the (op) triggers - LC
            if (gamepad2.right_trigger > 0.5){
                intakeRoller.setPower(-1);
            } else if (gamepad2.left_trigger > 0.5) {
                intakeRoller.setPower(1);
            } else {
                intakeRoller.setPower(0);
            }

            //set power and control of the motor for extendy to the (op) dpad L and R - LC
            if (gamepad2.dpad_right){
                hiExtend.setPower(1);
            } else if (gamepad2.dpad_left) {
                hiExtend.setPower(-1);
            } else {
                hiExtend.setPower(0);
            }

            //runs the hiJoint to -1100 when press and holding a calls in the file hiJointPIDController - LC 12/9
            if (gamepad2.a) {
                hiJointPIDController.setTarget(-1100);
                hiJointPIDController.update();
            }
            //runs the hiJoint to -400 when press and holding b calls in the file hiJointPIDController - LC 12/9
            if (gamepad2.b) {
                hiJointPIDController.setTarget(-400);
                hiJointPIDController.update();
            }

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftfrontPower, rightfrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftbackPower, rightbackPower);
            telemetry.addData("ViperSlide Power","%4.2f", vSPower);
            //telemetry.addData("Claw Power","%4.2f", clawPower);
            telemetry.update();

        }
    }
    private void hold() {
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        leftviperSlide.setPower(0);
        rightviperSlide.setPower(0);
        hiJoint.setPower(1);
        hiExtend.setPower(0);
        claw.setPower(0);
        intakeRoller.setPower(0);
    }

}






