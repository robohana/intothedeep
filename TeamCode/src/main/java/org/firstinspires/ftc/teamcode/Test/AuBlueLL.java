package org.firstinspires.ftc.teamcode.Test;

import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajSeq;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory1;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory2;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory3;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory4;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory5;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory6;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory7;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory8;
import static org.firstinspires.ftc.teamcode.Test.Trajectories.trajectory9;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ClawController;
import org.firstinspires.ftc.teamcode.Constants.hiJointPIDController;
import org.firstinspires.ftc.teamcode.Constants.vsPIDController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AuBlueLL")//, preselectTeleOp = "CompTeleOp")
public class AuBlueLL  extends LinearOpMode {
    private DcMotor hiJoint;
    public DcMotor leftviperSlide;
    public DcMotor rightviperSlide;
    public CRServo claw;
    public static double ARM_P = 0.005, ARM_I = 0, ARM_D = 0.0009, ARM_F = 0.1;
    private static final double ARM_TICKS_PER_DEGREE = 5281.1 / 180.0;

    public static double VS_P = 0.01, VS_I = 0, VS_D = 0.0001, VS_F = 0.1;
    private static final double VS_TICKS_PER_DEGREE = 537.7 / 180.0;

    private SampleMecanumDrive drive;
    public org.firstinspires.ftc.teamcode.Constants.hiJointPIDController hiJointPIDController;
    public org.firstinspires.ftc.teamcode.Constants.vsPIDController vsPIDController;
    public ClawController clawController;

    private Limelight3A limelight;

    boolean switchToPlanB = false;
    boolean continueWithPlanA = false;

    public void moveJointToTarget(
            int targetPosition,
            long timeoutMillis,
            double tolerance
    ) {
        hiJointPIDController.setTarget(targetPosition); // Set the PID target - LC 12/13
        long startTime = System.currentTimeMillis(); // Record start time - LC 12/13

        // Loop until the target is reached or timeout expires - LC 12/13
        while (opModeIsActive() &&
                Math.abs(hiJoint.getCurrentPosition() - hiJointPIDController.getTarget()) > tolerance &&
                (System.currentTimeMillis() - startTime) < timeoutMillis) {

            hiJointPIDController.update(); // Update PID - LC 12/13

            // Telemetry for debugging
            telemetry.addData("hiJoint Position", hiJoint.getCurrentPosition());
            telemetry.addData("hiJoint Target", hiJointPIDController.getTarget());
            telemetry.addData("Elapsed Time", (System.currentTimeMillis() - startTime) / 1000.0);
            telemetry.update();
        }

        hiJoint.setPower(0); // Stop the joint - LC 12/13
    }
    public void moveViperSlideToTarget(
            int targetPosition,
            double tolerance
    ) {
        vsPIDController.setTarget(targetPosition); // Set the PID target - LC 12/13
        long startTime = System.currentTimeMillis(); // Record start time - LC 12/13

        // Loop until the target is reached or timeout expires- LC 12/19
        while (opModeIsActive() &&
                Math.abs(leftviperSlide.getCurrentPosition() - vsPIDController.getTarget()) > tolerance) {

            vsPIDController.update(); // Update PID

            // Telemetry for debugging - LC 12/13
            telemetry.addData("LeftVS Position", leftviperSlide.getCurrentPosition());
            telemetry.addData("VS Target", vsPIDController.getTarget());
            telemetry.update();
        }

        // Stop both viper slides - 12/13
        leftviperSlide.setPower(0);
        rightviperSlide.setPower(0);
    }

    public void Plan_A() {
        while (opModeIsActive()) {
            moveJointToTarget(-600, 15000, 50);

            // runs us forward to the chambers in order to have the specimens, f: 22 - LC 12/10
            drive.followTrajectory(trajectory1);

            // puts the vs up high enough so we are in position to hang a sample once we move slightly forward - LC 12/10
            // Move viper slides to position 3500 with a tolerance of 50 - LC 12/13
            moveViperSlideToTarget(3000, 50);

            //gets us closer to the bar once our vs are up so that we can hang the specimen, f:8 - LC 12/10
            drive.followTrajectory(trajectory2);

            //open claw to release specimen - LC 12/13
            clawController.open();
            sleep(1000);

            // reverse so that we can lower the vs and be clear of the bars, b:8 - LC 12/10
            drive.followTrajectory(trajectory3);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13
            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(500);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            //run trajectory sequence where we move samples to the observation zone for the human player - LC 12/10
            drive.followTrajectorySequence(trajectory4);

            clawController.close();
            sleep(1000);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13
            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, just to get off the wall - LC 12/13
            moveViperSlideToTarget(1000, 50);

            drive.followTrajectory(trajectory9);
        }
    }
    /*public void Plan_A_Cont(){
        while (opModeIsActive()) {
            drive.followTrajectorySequence(trajectory5);

            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, up to the chamber - LC 12/13
            moveViperSlideToTarget(3500, 50);

            drive.followTrajectory(trajectory6);

            //open claw to release specimen - LC 12/13
            clawController.open();
            sleep(1000);

            drive.followTrajectory(trajectory7);

            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(700);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            drive.followTrajectorySequence(trajectory8);
        }
    }*/
    public void Plan_B() {
        ElapsedTime timer = new ElapsedTime(); // Create a timer to track elapsed time
        timer.reset(); // Start the timer

        while (opModeIsActive()&& timer.seconds() < 5) {
            telemetry.addData("Now in Plan B", "");
            telemetry.update();

            drive.followTrajectorySequence(trajSeq);
            switchToPlanB = false;
        }
        if (timer.seconds() >= 10) {
            telemetry.addData("Plan B Timeout", "Time limit reached (30 seconds)");
            telemetry.update();
        }
    }

    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        claw = hardwareMap.get(CRServo.class, "claw");

        hiJoint = hardwareMap.get(DcMotor.class, "hiJoint");
        leftviperSlide = hardwareMap.get(DcMotor.class, "leftviperSlide");
        rightviperSlide = hardwareMap.get(DcMotor.class, "rightviperSlide");

        hiJointPIDController = new hiJointPIDController(hiJoint, ARM_P, ARM_I, ARM_D, ARM_F, ARM_TICKS_PER_DEGREE);
        vsPIDController = new vsPIDController(leftviperSlide, rightviperSlide, VS_P, VS_I, VS_D, VS_F, VS_TICKS_PER_DEGREE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(50);

        limelight.pipelineSwitch(2);

        limelight.start();

        Pose2d startPose = new Pose2d(-1, 50, -1.5707963268);
        drive.setPoseEstimate(startPose);
        Trajectories.initialize(drive, startPose);

        clawController = new ClawController(claw);

        long startTime = System.currentTimeMillis(); // Record the starting time - LC 12/18

        hiJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hiJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftviperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftviperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        double ta = result.getTa();

        telemetry.addData("Limelight Status", "Pipeline: %d", status.getPipelineIndex());

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        if (isStopRequested()) return;

        /*
         * Runs sequence, runs the joint angles up slightly to be out of the way of the viper
         * slides then it runs through the trajectory seq made above - LC 12/9
         */
        while (opModeIsActive()) {

            moveJointToTarget(-600, 15000, 50);

            // runs us forward to the chambers in order to have the specimens, f: 22 - LC 12/10
            drive.followTrajectory(trajectory1);

            // puts the vs up high enough so we are in position to hang a sample once we move slightly forward - LC 12/10
            // Move viper slides to position 3500 with a tolerance of 50 - LC 12/13
            moveViperSlideToTarget(3000, 50);

            //gets us closer to the bar once our vs are up so that we can hang the specimen, f:8 - LC 12/10
            drive.followTrajectory(trajectory2);

            //open claw to release specimen - LC 12/13
            clawController.open();
            sleep(1000);

            // reverse so that we can lower the vs and be clear of the bars, b:8 - LC 12/10
            drive.followTrajectory(trajectory3);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13
            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(500);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            //run trajectory sequence where we move samples to the observation zone for the human player - LC 12/10
            drive.followTrajectorySequence(trajectory4);

            clawController.close();
            sleep(1000);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13
            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, just to get off the wall - LC 12/13
            moveViperSlideToTarget(1000, 50);

            drive.followTrajectory(trajectory9);

            while (ta != 0 && !switchToPlanB) {  // If specimen is found and we're not already in Plan B
                // Plan A logic
                Plan_A();
            }
            if (ta == 0 && !switchToPlanB) {  // If no specimen is found, switch to Plan B
                telemetry.addData("No Specimen", "Switching to Plan B");
                telemetry.update();
                switchToPlanB = true;
                Plan_B();  // Run Plan B
            } else {
                switchToPlanB = true;
            }
            while (switchToPlanB){
                drive.followTrajectorySequence(trajSeq);
            }

            // Continuously check if we should revert to Plan A (e.g., if the specimen is found later)
           /* while (ta != 0 && switchToPlanB) {
                switchToPlanB = false;  // Reset flag to switch back to Plan A
                Plan_A();  // Continue Plan A if specimen is found after Plan B
            }*/

            // Allow the robot to do other tasks if needed (e.g., idle)
            idle();

            /*while (!switchToPlanB) {
                Plan_A();
                sleep(500); // Ensure there's a slight delay after Plan A
            }
            while (ta == 0) {
                telemetry.addData("No Specimen", "Switching to Plan B");
                telemetry.update();
                switchToPlanB = true;  // Set the flag to true to transition to Plan B
            }*/
            /*while (!(ta == 0)){
                telemetry.addData("Have Specimen", "Continuing with Plan A");
                telemetry.update();
                continueWithPlanA = true;
            }*/
            /*while (switchToPlanB) {
                Plan_B();  // Run Plan B
                switchToPlanB = false;  // Reset the flag to avoid going back to Plan B indefinitely
            }*/
            /*while (continueWithPlanA){
                Plan_A_Cont();  // Call Plan B if the flag is set - LC 12/30
                continueWithPlanA = false;  // Reset the flag to avoid an infinite loop - LC 12/30
            }*/

            // Debugging info
            telemetry.addData("Limelight", "Target Area: %.2f", ta);
            telemetry.update();
            /*Plan_A();
            sleep (500);
            // Check condition to break the loop - LC 12/30
            while (ta == 0) {
                telemetry.addData("No Specimen", "Switching to Plan B");
                telemetry.update();
                switchToPlanB = true;  // Set the flag to true - LC 12/30
                break;  // Exit the current loop and move to Plan B - LC 12/30
            }
            while (!(ta == 0)){
                telemetry.addData("Have Specimen", "Continuing with Plan A");
                telemetry.update();
                continueWithPlanA = true;
                Plan_A_Cont();
            }
            while (switchToPlanB) {
                Plan_B();  // Call Plan B if the flag is set - LC 12/30
                switchToPlanB = false;  // Reset the flag to avoid an infinite loop - LC 12/30

            }
            while (continueWithPlanA){
                Plan_A_Cont();  // Call Plan B if the flag is set - LC 12/30
                continueWithPlanA = false;  // Reset the flag to avoid an infinite loop - LC 12/30
            }*/


            /*if (!(ta == 0)) {
                drive.followTrajectory(trajectory1);
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Specimen:", "Loaded" );
                telemetry.addData("Target", "Detected with area: %.2f", ta);

            } else {
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Specimen:", "Not Loaded" );
            }
            telemetry.update();*/

            //while (!isStopRequested()) idle();

        }
        limelight.stop();

    }

}
