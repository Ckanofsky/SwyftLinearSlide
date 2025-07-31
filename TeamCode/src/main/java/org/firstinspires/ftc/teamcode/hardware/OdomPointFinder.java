package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "OdomPointFinder")


public class OdomPointFinder extends LinearOpMode {
    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        robot.init();

        robot.drive.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.drive.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.drive.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.drive.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();

        robot.myOtos.resetTracking();
        robot.myOtos.calibrateImu();
        robot.imu.resetHeading();
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (true) {

            pos = robot.myOtos.getPosition();

            pos.x *= -1;
            pos.y *= -1;

            double strafe = 0;
            double forward = 0;
            double turn = 0;
            if (turn >= 180) {
                turn -= 360;
            }
            if (turn <= -180) {
                turn += 360;
            }

            turn *= -1;


            // bump up these constants slightly if bot becomes heavy
            double P = 0;
            strafe *= P;
            //strafe += Math.copySign(0.11, strafe);
            forward *= P;
            //forward += Math.copySign(0.11, forward);

            turn *= P * (2 / 3f);


            double botHeading = Math.toRadians(pos.h);

            double rotStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);


            telemetry.addData("Robot x pos current", pos.x);
            telemetry.addData("Robot y pos current", pos.y);

            telemetry.addData("Robot strafe", strafe);
            telemetry.addData("Robot move forward", forward);

            telemetry.addData("robot heading ", (pos.h));

            telemetry.update();

            robot.drive.drivePower(rotForward, rotStrafe, turn);
        }

    }
}