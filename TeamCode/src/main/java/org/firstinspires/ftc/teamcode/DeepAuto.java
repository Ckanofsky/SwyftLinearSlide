package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Enhanced Autonomous", group = "Autonomous")
public class DeepAuto extends LinearOpMode {

    private AutonomousMovement autoMove;

    @Override
    public void runOpMode() {
        // Initialize the movement system
        autoMove = new AutonomousMovement(this);

        // Optional: Tune parameters for your robot
        autoMove.setAcceleration(1.5);
        autoMove.setDeceleration(2.0);
        autoMove.setPIDConstants(0.058, 0.11);
        autoMove.setTolerances(1.0, 2.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence


            // Move 1: Linear movement with turn at start
            autoMove.enhancedMovement(
                    0, 0,                    // start position (current position)
                    24, 0,                   // target position
                    0, 45,                   // start heading, end heading
                    0.8,                     // max speed
                    "linear_turn_start",     // path type
                    1.0                      // position tolerance
            );

            sleep(2000); // Brief pause



            // Move 3: Linear with distributed turn
            autoMove.enhancedMovement(
                    24, 0,                  // start position
                    0, 0,                   // target position
                    45, 235,                // start heading, end heading
                    0.7,                     // max speed
                    "linear_turn_distributed", // path type
                    1.0                      // position tolerance
            );


            sleep(2000);

            // Move 2: Smooth bezier curve
            autoMove.enhancedMovement(
                    0, 0,                   // start position
                    24, 0,                  // target position
                    235, 45,                 // start heading, end heading
                    0.6,                     // max speed
                    "bezier",                // path type
                    1.5                      // position tolerance
            );

            sleep(2000);

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }
}