package org.firstinspires.ftc.teamcode.Test;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
@TeleOp(name = "Limelight Color Detection Linear", group = "Examples")
public class ColorLL extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
            // Initialize your Limelight camera (assuming Limelight is connected to the Robot Controller)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(500);

        limelight.pipelineSwitch(2);

        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            /*telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            telemetry.update();*/

            /*List<ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Name", "%s",
                    status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                telemetry.update();
            }*/
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                List<ColorResult> colorResults = result.getColorResults();
                if (colorResults != null && !colorResults.isEmpty()) {
                    for (ColorResult cr : colorResults) {
                        if (cr == null) {
                            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                                    status.getTemp(), status.getCpu(), (int) status.getFps());
                            telemetry.addData("Color Result", "Found a null color result");
                            telemetry.addData("Specimen:", "Not Loaded" );
                            telemetry.update();

                        } else {
                            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                                    status.getTemp(), status.getCpu(), (int) status.getFps());
                            telemetry.addData("Specimen:", "Loaded" );
                            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                            telemetry.update();

                        }
                    }
                } else {
                    telemetry.addData("Color Results", "No color results available");
                    telemetry.update();
                }
            } else {
                telemetry.addData("Error", "No result available from Limelight");
                telemetry.update();
            }
            //telemetry.update();
        }
        limelight.stop();
    }
}
