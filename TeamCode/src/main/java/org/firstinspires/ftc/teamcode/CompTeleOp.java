package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.TouchSensor;



/*
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Score;
import com.qualcomm.robotcore.hardware.HardwareMap;
*/



@TeleOp(name="CompTeleOp")
public class CompTeleOp extends LinearOpMode {
    //decale all public guys
    public DcMotor backleftDrive;
    public DcMotor backrightDrive;
    public DcMotor frontleftDrive;
    public DcMotor frontrightDrive;

    public CRServo leftBrush;
    public CRServo rightBrush;
    public Servo intakeWrist;
    public DcMotor horizontalviperSlide;

    public DcMotor leftviperSlide;
    public DcMotor rightviperSlide;
    public CRServo claw;


    public Servo dumpAngle;
    public DcMotor verticalviperSlide;

    double vvSPower;
    //double vSPower
    //double hvSPower;
    double max;
    public TouchSensor vvStouchSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        double t = getRuntime();


        //handware map for Drivetrain - LC
        backleftDrive = hardwareMap.get(DcMotor.class, "backleftDrive");
        backrightDrive = hardwareMap.get(DcMotor.class, "backrightDrive");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontleftDrive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontrightDrive");

        //hardware map for Intake - LC
        leftBrush = hardwareMap.get(CRServo.class, "leftBrush");
        rightBrush = hardwareMap.get(CRServo.class, "rightBrush");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        horizontalviperSlide = hardwareMap.get(DcMotor.class, "horizontalviperSlide");

        //hardware map for Score - LC
        dumpAngle = hardwareMap.get(Servo.class, "dumpAngle");
        verticalviperSlide = hardwareMap.get(DcMotor.class, "verticalviperSlide");
        //DcMotor encodervvS = hardwareMap.dcMotor.get("verticalviperSlide");
        vvStouchSensor = hardwareMap.get(TouchSensor.class, "vvStouchSensor");

        leftviperSlide = hardwareMap.get(DcMotor.class, "leftviperSlide");
        rightviperSlide = hardwareMap.get(DcMotor.class, "rightviperSlide");
        claw = hardwareMap.get(CRServo.class, "claw");


        //this sets the direction the motor will spin for the drive wheels - LC
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftviperSlide.setDirection(DcMotor.Direction.FORWARD);
        rightviperSlide.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if(opModeIsActive()) {
                if (gamepad1.dpad_left) {         // go up - LC
                    claw.setPower(0);
                } else {
                    claw.setPower(1);

                }
            }

            double power = -gamepad2.left_stick_y;
            leftviperSlide.setPower(power);
            rightviperSlide.setPower(power);


            //Drivetrain();
            //Score();

            //Controlls for gamepad 1 (driver) - LC
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

            // Get the current position of the vvS - LC
            //double position = encodervvS.getCurrentPosition();

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftfrontPower, rightfrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftbackPower, rightbackPower);
            //telemetry.addData("vvS Touch Sensor", vvStouchSensor.getValue());
            //telemetry.addData ("vvS Power", vvSPower);
            //telemetry.addData ("hvs Power", hvSPower);
            //telemetry.addData ("Encoder Position", position);
            telemetry.addData("t", t);
            telemetry.addData("rt-t", getRuntime()-t);
            telemetry.update();

        }
    }

}






