package org.firstinspires.ftc.teamcode.Test;

// I had to import every one of these :( - LC 12/30/24

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
    //TODO #1: Make a Red Version of this after it is done and we know it works. OR for saturday 1/4/25 - LC 12/30/24
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

    //TODO #2: MAKE SURE AFTER YOU RUN THE METHOD AT THE END OF IT THAT YOU SET THE FLAG BACK TO FALSE :) - LC 12/30/24
    boolean switchToPlanB = false;
    boolean continueWithPlanA = false;

    public void moveJointToTarget(
            int targetPosition,
            long timeoutMillis,
            double tolerance
    ) {
        hiJointPIDController.setTarget(targetPosition); // Set the PID target - LC 12/13/24
        long startTime = System.currentTimeMillis(); // Record start time - LC 12/13/24

        // Loop until the target is reached or timeout expires - LC 12/13/24
        while (opModeIsActive() &&
                Math.abs(hiJoint.getCurrentPosition() - hiJointPIDController.getTarget()) > tolerance &&
                (System.currentTimeMillis() - startTime) < timeoutMillis) {

            hiJointPIDController.update(); // Update PID - LC 12/13/24

            // Telemetry for debugging - LC 12/13/24
            telemetry.addData("hiJoint Position", hiJoint.getCurrentPosition());
            telemetry.addData("hiJoint Target", hiJointPIDController.getTarget());
            telemetry.addData("Elapsed Time", (System.currentTimeMillis() - startTime) / 1000.0);
            telemetry.update();
        }
        hiJoint.setPower(0); // Stop the joint - LC 12/13/24
    }
    public void moveViperSlideToTarget(
            int targetPosition,
            double tolerance
    ) {
        vsPIDController.setTarget(targetPosition); // Set the PID target - LC 12/13/24
        long startTime = System.currentTimeMillis(); // Record start time - LC 12/13/24

        // Loop until the target is reached or timeout expires- LC 12/19/24
        while (opModeIsActive() &&
                Math.abs(leftviperSlide.getCurrentPosition() - vsPIDController.getTarget()) > tolerance) {

            vsPIDController.update(); // Update PID - LC 12/13.24

            // Telemetry for debugging - LC 12/13/24
            telemetry.addData("LeftVS Position", leftviperSlide.getCurrentPosition());
            telemetry.addData("VS Target", vsPIDController.getTarget());
            telemetry.update();
        }

        // Stop both viper slides - LC 12/13/24
        leftviperSlide.setPower(0);
        rightviperSlide.setPower(0);
    }

    /*
    * This is the start of my two specimen auto with using limelight. It starts just like the previous two specimen auto seen and tested in Meet 3 but now we have LIGHTS [They're not that bright and hard to see (be we have LIGHTS)]. I use PID controllers for hiJoint and the viper slides. With the viper slides I use gravity so that I don't actually have to pull them down as much and spend more time on that. This part of the code hangs one specimen then goes over and grabs another one off the human player wall. The code then ends there so that I can check to see if we have a specimen in our buttery paws (idfk). - LC 12/30/24 */
    // TODO #3: Goal would be to just have a break command in here but that wasn't working earlier the robot was not being very cooperative. - LC 12/30/24
    public void Plan_A() {
        while (opModeIsActive()) {
            //Start of 2 Specimen Auto - LC 12/30/24
            moveJointToTarget(-600, 15000, 50);

            // runs us forward to the chambers in order to have the specimens, f: 22 - LC 12/10/24
            // F = 22 - LC 12/30/24
            drive.followTrajectory(trajectory1);

            // puts the vs up high enough so we are in position to hang a sample once we move slightly forward - LC 12/10/24
            // Move viper slides to position 3500 with a tolerance of 50 - LC 12/13/24
            moveViperSlideToTarget(3000, 50);

            //gets us closer to the bar once our vs are up so that we can hang the specimen, f:8 - LC 12/10/24
            // F = 8 - LC 12/30/24
            drive.followTrajectory(trajectory2);

            //open claw to release specimen - LC 12/13/24
            clawController.open();
            sleep(1000);

            // reverse so that we can lower the vs and be clear of the bars, b:8 - LC 12/10/24
            // B = 8 - LC 12/30/24
            drive.followTrajectory(trajectory3);

            //Move viper slides to position -600 with a 15-second timeout and a tolerance of 50 - LC 12/30/24
            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13/24
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(500);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            //run trajectory sequence where we move samples to the observation zone for the human player - LC 12/10/24
            // T = -180, SL = 52, F = 25 - LC 12/30/24
            drive.followTrajectorySequence(trajectory4);

            clawController.close();
            sleep(1000);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13/24
            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, just to get off the wall - LC 12/13/24
            moveViperSlideToTarget(1000, 50);

            // B = 10 - LC 12/30/24
            drive.followTrajectory(trajectory9);
        }
    }

    /*
    * Part Two of Plan A (this will just be one with a break if statement in it once the robot starts behaving again. This is the rest of the two specimen auto tested in Meet 3. It can definitely be tweaked further into working better/make it more  reliable. If ran through Part 1 & 2 of A then it would be the same code as the other 2 specimen auto code just this one has traj. 5 split where the first movement of reversing now is traj. 9. It LOVES to run this part of the code the robot was really being a poo poo head on 12/30/24, he did not want to listen. The numbers probably need to be dialed in better. - LC 12/30/24
    * */
    // TODO/TA (Think About) #4: Maybe pushing one specimen to the HP so that our alliance partner is still able to have an alliance specific specimen if they want one - LC 12/30/24
    public void Plan_A_Cont(){
        while (opModeIsActive()) {
            // T = -180, F = 18, SL = 60 - LC 12/30/24
            drive.followTrajectorySequence(trajectory5);

            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, up to the chamber - LC 12/13/24
            moveViperSlideToTarget(3500, 50);

            // F = 8 - LC 12/30/24
            drive.followTrajectory(trajectory6);

            //open claw to release specimen - LC 12/13/24
            clawController.open();
            sleep(1000);

            // B = 8 - LC 12/30/24
            drive.followTrajectory(trajectory7);

            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13/24
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(700);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            // SR = 60, B = 12 - LC 12/30/24
            drive.followTrajectorySequence(trajectory8);
            continueWithPlanA = false; //reset flag - LC 12/30/24
        }
    }
    /*
    * Plan B, our back up plan for if we can not successfully get the butter off the wall into our very buttery paws. - LC 12/30/24
    * (Smokey is getting dog hair all over my keyboard) - LC 12/30/24
    * Our back up plan for this situation is that we go to our alliance specific colored samples and push two of those in 1 by 1. This was also displayed in Meet 3 where we unlike several other teams (attempting to do a sample push auto) did not get any penalties during auto (from OUR auto). This is tested auto this good.Values in the trajectory sequence are completely guessed though so we are going to have to dial that in. We are having issues getting stuck in a loop here (that bad) I added in the timer to hopefully prevent us from getting any more BAD BAD RED DS WARNINGS that make the robot have to restart. That very bad - LC 12/30/24
    * */
    //TODO #5 : MAKE IT STOP DEATH LOOPING - LC 12/30/24
    public void Plan_B() {
        // don't know if this is going to be okay having a second timer in the code but we will see. Named it diff to see if that help - LC 12/30/24
        ElapsedTime timer2 = new ElapsedTime(); // Create a timer to track elapsed time - LC 12/30/24
        timer2.reset(); // Start the timer2 - LC 12/30/24

        while (opModeIsActive()&& timer2.seconds() < 5) {
            telemetry.addData("Now in Plan B", "");
            telemetry.update();
            // TODO #6: Dial in the trajSeq values - LC 12/30/24
            // SL = 10, B = 30, SL = 7, F = 45, B = 50, SL = 7, F = 45 - LC 12/30/24
            drive.followTrajectorySequence(trajSeq);
            switchToPlanB = false; // reset flag - LC 12/30/24
        }
        //TODO #7: Adjust times as needed after running some time trials to see how long it will take to run "trajSeq" - LC 12/30/24
        if (timer2.seconds() >= 10) {
            telemetry.addData("Plan B Timeout", "Time limit reached (30 seconds)");
            telemetry.update();
            switchToPlanB = false; // reset flag - LC 12/30/24

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

        long startTime = System.currentTimeMillis(); // Record the starting time - LC 12/18/24

        hiJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hiJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftviperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftviperSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        double ta = result.getTa();

        waitForStart();
        telemetry.addData("Limelight Status", "Pipeline: %d", status.getPipelineIndex());

        ElapsedTime timer1 = new ElapsedTime();
        timer1.reset();

        if (isStopRequested()) return;

        /*
         * Runs sequence, runs the joint angles up slightly to be out of the way of the viper slides then it runs through the trajectory seq made above - LC 12/9/24
         */
        while (opModeIsActive()) {
            // This is plan A Can run this or the method Plan_A() above - LC 12/30/24
            /*moveJointToTarget(-600, 15000, 50);

            // runs us forward to the chambers in order to have the specimens, f: 22 - LC 12/10/24
            drive.followTrajectory(trajectory1);

            // puts the vs up high enough so we are in position to hang a sample once we move slightly forward - LC 12/10/24
            // Move viper slides to position 3500 with a tolerance of 50 - LC 12/13/24
            moveViperSlideToTarget(3000, 50);

            //gets us closer to the bar once our vs are up so that we can hang the specimen, f:8 - LC 12/10/24
            drive.followTrajectory(trajectory2);

            //open claw to release specimen - LC 12/13/24
            clawController.open();
            sleep(1000);

            // reverse so that we can lower the vs and be clear of the bars, b:8 - LC 12/10/24
            drive.followTrajectory(trajectory3);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13/24
            moveJointToTarget(-600, 15000, 50);

            //runs vs down to 0 position with help from gravity then set power to zero - LC 12/13/24
            leftviperSlide.setPower(-1);
            rightviperSlide.setPower(1);
            sleep(500);

            leftviperSlide.setPower(0);
            rightviperSlide.setPower(0);

            //run trajectory sequence where we move samples to the observation zone for the human player - LC 12/10/24
            drive.followTrajectorySequence(trajectory4);

            clawController.close();
            sleep(1000);

            //Move viper slides to position -500 with a 15-second timeout and a tolerance of 100 - LC 12/13/24
            moveJointToTarget(-600, 15000, 50);

            // Move viper slides to position 100 with a tolerance of 50, just to get off the wall - LC 12/13/24
            moveViperSlideToTarget(1000, 50);

            drive.followTrajectory(trajectory9);*/

            /**Logic, hopefully it works now. and not continues to play A if i have commented the code out everywhere and I don't know where it is getting the information from - LC 12/30/24, I'm losing my sanity over here. PLZ send help**/

            //TODO #8: figure out if the camera can see if a butter is on the wall before it goes in to get one. We could have a ditch then and we can also check after we have reversed to see if it was dropped. - LC 12/30/24
            //TODO #9: add in a ditch during/at the very start of the match for if we accidentally drop it or something else happens on our approach - LC 12/30/24
            if (ta != 0 && !switchToPlanB) {    // If specimen is found and we're not already in Plan B - LC 12/30/24
                // Plan A logic - LC 12/30/24
                telemetry.addData("Start", "Playing Plan A");
                telemetry.update();
                Plan_A();
            }
            sleep (500);
            if (ta == 0 && !switchToPlanB) {  // If no specimen is found, switch to Plan B - LC 12/30/24
                telemetry.addData("No Specimen", "Switching to Plan B");
                telemetry.update();
                switchToPlanB = true;
                sleep(500);
                Plan_B();  // Run Plan B - LC 12/30/24
            } else if (ta != 0 && !switchToPlanB) { //if one condition is not met then continue A - LC 12/30/24
                continueWithPlanA = true;
                Plan_A_Cont(); // Run Plan A_Cont - LC 12/30/24
            } else{
                sleep(1000); // just do nothing if you don't have it, you will be parked so we good - LC 12/30/24
            }


            // Continuously check if we should revert to Plan A (e.g., if the specimen is found later) - LC 12/30/24
           /* while (ta != 0 && switchToPlanB) {
                switchToPlanB = false;  // Reset flag to switch back to Plan A
                Plan_A();  // Continue Plan A if specimen is found after Plan B
            }*/

            // Allow the robot to do other tasks if needed (e.g., idle) - LC 12/30/24
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
                Plan_A_Cont();  // Call Plan B if the flag is set - LC 12/30/24
                continueWithPlanA = false;  // Reset the flag to avoid an infinite loop - LC 12/30/24
            }*/

            // Debugging info - LC 12/30/24
            telemetry.addData("Limelight", "Target Area: %.2f", ta);
            telemetry.update();

            /*Plan_A();
            sleep (500);
            // Check condition to break the loop - LC 12/30/24
            while (ta == 0) {
                telemetry.addData("No Specimen", "Switching to Plan B");
                telemetry.update();
                switchToPlanB = true;  // Set the flag to true - LC 12/30/24
                break;  // Exit the current loop and move to Plan B - LC 12/30/24
            }
            while (!(ta == 0)){
                telemetry.addData("Have Specimen", "Continuing with Plan A");
                telemetry.update();
                continueWithPlanA = true;
                Plan_A_Cont();
            }
            while (switchToPlanB) {
                Plan_B();  // Call Plan B if the flag is set - LC 12/30/24
                switchToPlanB = false;  // Reset the flag to avoid an infinite loop - LC 12/30/24

            }
            while (continueWithPlanA){
                Plan_A_Cont();  // Call Plan B if the flag is set - LC 12/30/24
                continueWithPlanA = false;  // Reset the flag to avoid an infinite loop - LC 12/30/24
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
        //ensure to stop the limelight - LC 12/30/24
        limelight.stop();

    }

}
