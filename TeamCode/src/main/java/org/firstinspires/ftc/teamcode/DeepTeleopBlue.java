package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.LinearSlides;
import org.firstinspires.ftc.teamcode.hardware.Robot;


@TeleOp(name = "Teleop")

//WHYYYYYYYYY :(
public class DeepTeleopBlue extends LinearOpMode{
    Robot robot = new Robot(this);
    double speedReduction = 1;


    final boolean driveActive = true;
    final boolean linearSlideActive = true;
    final boolean intakeActive = false;
    final boolean outtakeActive = false;


    @Override
    public void runOpMode() {
        robot.init();
        waitForStart();
        resetRuntime();
        robot.imu.resetHeading();
        while (opModeIsActive()) { // Start of Teleop


//Drive code
            if(driveActive) {
                if (gamepad1.back) {
                    robot.imu.resetHeading();
                }
                double botHeading = robot.imu.getHeading();
                double y = -gamepad1.left_stick_y / speedReduction; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x / speedReduction;
                // Rotate the movement direction with the bot rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                robot.drive.drivePower(rotY, rotX, gamepad1.right_stick_x / speedReduction);
            }


//Linear Slide Code
            if (linearSlideActive) {
                robot.linearSlides.manualLinearSlide(gamepad1.dpad_up, gamepad1.dpad_down); //Manual up and down
            }


//Intake Code
            if (intakeActive){
                robot.intake.intakeManual(gamepad1.right_trigger, gamepad1.left_trigger); //Manual intake and outake
            }


//Outake Code
            if (outtakeActive){
                robot.outtake.outtake(gamepad1.a);            //Manual outtake open and close
                robot.outtake.outtakeRotate(gamepad1.x);      //Manual outtake rotate and un-rotate
            }


//Telemetry
            telemetry.addData("Drive State: ", driveActive);
            telemetry.addData("LinearSlide State: ", linearSlideActive);
            telemetry.addData("Intake State: ", intakeActive);
            telemetry.addData("Outake State: ", outtakeActive);
            telemetry.addData("LinearSlidePosition: ", robot.linearSlides.getSlidePosition());
            telemetry.addData("SlideMagnetSensor: ", robot.linearSlides.slideSensorActive());
            telemetry.addData("SlideMagnetSensor: ", robot.intake.intakeSensorActive());
            telemetry.addData("SlideMagnetSensor: ", robot.outtake.outtakeSensorActive());
            telemetry.update();
        }
    }
}