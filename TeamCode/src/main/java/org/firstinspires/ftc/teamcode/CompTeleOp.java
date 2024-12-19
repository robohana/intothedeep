package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ClawController;
import org.firstinspires.ftc.teamcode.Constants.DriveController;
import org.firstinspires.ftc.teamcode.Constants.IntakeRollerController;
import org.firstinspires.ftc.teamcode.Constants.hiExtendController;
import org.firstinspires.ftc.teamcode.Constants.hiJointPIDController;
import org.firstinspires.ftc.teamcode.Constants.vsPIDController;
import com.qualcomm.robotcore.hardware.VoltageSensor;


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
    public static final int ARM_TARGET1 = -1100;
    public static final int ARM_TARGET2 = -400;

    public static double VS_P = 0.01, VS_I = 0, VS_D = 0.0001, VS_F = 0.1;
    private static final double VS_TICKS_PER_DEGREE = 5281.1 / 180.0;
    private static final int VS_TARGET = -300;
    public vsPIDController vsPIDController;
    public ClawController clawController;
    public IntakeRollerController intakerollerController;
    public hiExtendController hiExtendController;
    private DriveController driveController;

    double scalingFactor = 0.5;

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

        //Controller instances - LC 12/10
        hiJointPIDController = new hiJointPIDController(hiJoint, ARM_P, ARM_I, ARM_D, ARM_F, ARM_TICKS_PER_DEGREE);
        vsPIDController = new vsPIDController(leftviperSlide, rightviperSlide, VS_P, VS_I, VS_D, VS_F, VS_TICKS_PER_DEGREE);
        clawController = new ClawController(claw);
        intakerollerController = new IntakeRollerController(intakeRoller);
        hiExtendController = new hiExtendController(hiExtend);
        driveController = new DriveController(frontleftDrive, frontrightDrive, backleftDrive, backrightDrive);

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

        //hiJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hiJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (hiJoint == null) {
            throw new NullPointerException("hiJoint is null. Check your configuration.");
        }
        telemetry.addData("hiJoint", hiJoint != null ? "Initialized" : "Null");
        telemetry.addData("hiJointPIDController", hiJointPIDController != null ? "Initialized" : "Null");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double voltage = getBatteryVoltage();
            double cpuUtilization = getCpuUtilization();

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
            //jointPower *= scalingFactor;
            hiJoint.setPower(jointPower);
            // Use the DriveController to handle driving - LC 12/10
            driveController.drive(gamepad1);

            /* set's it so that the butter gripper will be right all the way open, left all the way
            close or prob just chill. hopefully just chill. This is also with a CR Servo so 1 and
            -1 are max power to the system. you can do half power but DON'T SET THE POWER -1<=X<=1
            OR I WILL FIND YOU!!- LC  */
            if (gamepad2.right_bumper) {         // go max open using clawController - LC 12/10
                clawController.open();
            } else if (gamepad2.left_bumper) {   // go to closed using clawController - LC 12/10
                clawController.close();
            } else {                             // go to no power using clawController - LC 12/10
                clawController.stop();
            }

            //set power and control of the servo for intake to the (op) triggers - LC
            if (gamepad2.right_trigger > 0.5){          // intake using intakerollerController - LC 12/10
                intakerollerController.intake();
            } else if (gamepad2.left_trigger > 0.5) {   // release using intakerollerController - LC 12/10
                intakerollerController.release();
            } else {                                    // stop using intakerollerController - LC 12/10
                intakerollerController.stop();
            }

            //set power and control of the motor for extendy to the (op) dpad L and R - LC
            if (gamepad2.dpad_right){               // extend hiExtend arm using hiExtendController - LC 12/10
                hiExtendController.extend();
            } else if (gamepad2.dpad_left) {        // retract hiExtend arm using hiExtendController - LC 12/10
                hiExtendController.retract();
            } else {                                // stop hiExtend arm using hiExtendController - LC 12/10
                hiExtendController.stop();
            }

            //runs the hiJoint to -1100 when press and holding a calls in the file hiJointPIDController - LC 12/9
            if (gamepad2.a) {
                hiJointPIDController.setTarget(ARM_TARGET1);
                hiJointPIDController.update();
            }
            //runs the hiJoint to -400 when press and holding b calls in the file hiJointPIDController - LC 12/9
            if (gamepad2.b) {
                hiJointPIDController.setTarget(ARM_TARGET2);
                hiJointPIDController.update();
            }

            telemetry.addData("Battery Voltage", voltage);
            telemetry.addData("hiJoint POS", hiJoint.getCurrentPosition());
            telemetry.addData("CPU Utilization", String.format("%.2f%%", cpuUtilization * 100));
            telemetry.addData("ViperSlide Power","%4.2f", vSPower);
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