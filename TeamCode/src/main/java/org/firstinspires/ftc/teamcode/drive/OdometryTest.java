package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name = "Odometry Test", group = "Test")
@Disabled
public class OdometryTest extends LinearOpMode{
        // Declare Odometry Driver
        GoBildaPinpointDriver odo;

        @Override
        public void runOpMode() throws InterruptedException {
            // Initialize hardware map to get odometry driver
            telemetry.addData("Status", "Initializing...");
            telemetry.update();

            try {
                odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
                odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
                odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
                odo.resetPosAndIMU();
                telemetry.addData("Status", "Odometry Initialized");
            } catch (Exception e) {
                telemetry.addData("Error", "Odometry Initialization Failed: " + e.getMessage());
            }

            telemetry.update();

            waitForStart();

            if (odo != null) {
                while (opModeIsActive()) {
                    odo.update();
                    try {
                        // Get the X-coordinate from the odometry
                        double xPosition = odo.getPosition().getX(DistanceUnit.MM);

                        // Send the X-coordinate value to telemetry
                        telemetry.addData("X Position (mm)", xPosition);

                    } catch (Exception e) {
                        telemetry.addData("Error", "Failed to get X Position: " + e.getMessage());
                    }

                    telemetry.update();

                    // Sleep to avoid flooding telemetry
                    sleep(100);
                }
            } else {
                telemetry.addData("Error", "Odometry hardware not found.");
                telemetry.update();
            }
        }
}