/*
    Hardware Map Layout  -  NamingOnConfig  -  (front of the robot is the side with Go-Rail, view from top down)
    Drive motors:           frontLeft, frontRight, backLeft, backRight
    IMU:                    imu
    Odometry:               odometry
    Intake motor:           intakeMotor
    Intake sensor:          intakeSensor
    LinearSlide motors:     leftLinearSlide, rightLinearSlide
    LinearSlide sensors:    leftSlideSensor, rightSlideSensor
    Outtake servos:         outtakeRotate, outtakeRelease
    Outtake sensor:         outtakeSensor
 */

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Robot {
    public LinearOpMode myOpMode;
    public HardwareMap hwMap;
    public Drive drive = new Drive(this);
    public IMU imu = new IMU(this);
    public SparkFunOTOS myOtos;
    public LinearSlides linearSlides = new LinearSlides(this);

    public Intake intake = new Intake(this);
    public Outtake outtake = new Outtake(this);

    public Robot(LinearOpMode OpMode) {
        this.myOpMode = OpMode;
    }

    public void configureOtos() {
        myOtos = hwMap.get(SparkFunOTOS.class, "odometry");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
    }

    public void init() {
        hwMap = myOpMode.hardwareMap;

        // This makes the code run faster:
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.init();
        imu.init();

        linearSlides.init();
        intake.init();
        outtake.init();

        configureOtos();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
}

