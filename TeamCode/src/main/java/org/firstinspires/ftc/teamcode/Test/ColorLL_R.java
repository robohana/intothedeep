package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Color Red", group = "Examples")
public class ColorLL_R extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
            // Initialize your Limelight camera (assuming Limelight is connected to the Robot Controller)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(50);

        limelight.pipelineSwitch(0);

        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();
            double ta = result.getTa();  // Get target area

            // If target area is zero, treat it as no target detected (empty target)
            if (ta == 0) {
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Specimen:", "Not Loaded" );
            } else {
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Specimen:", "Loaded" );
                telemetry.addData("Target", "Detected with area: %.2f", ta);
            }
            telemetry.update();
        }
        limelight.stop();
    }
}
