package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    Robot robot;
    private Servo outtakeRotate;
    private Servo outtakeRelease;

    private TouchSensor outtakeSensor;

    Outtake(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        outtakeRotate = robot.hwMap.get(Servo.class, "outtakeRotate");
        outtakeRotate.setPosition(0.5);
        outtakeRelease = robot.hwMap.get(Servo.class, "outtakeRelease");
        outtakeRelease.setPosition(0.5);
        outtakeSensor = robot.hwMap.get(TouchSensor.class, "outtakeSensor");
    }

    boolean outtakeClosed = false;
    ElapsedTime timer = new ElapsedTime(); //Prevents double clicks
    public void outtake(boolean buttonPressed){
        if (buttonPressed && !outtakeClosed && (timer.milliseconds() >= 100)) {
            outtakeRelease.setPosition(.25); //Close to grab item
            outtakeClosed = true;
            timer.reset();
        } else if (buttonPressed && outtakeClosed && (timer.milliseconds() >= 100)){
            outtakeRelease.setPosition(.75); //Open to release item
            outtakeClosed = false;
            timer.reset();
        }
    }

    boolean outtakeRotated = true;
    ElapsedTime timer2 = new ElapsedTime(); //Prevents double clicks
    public void outtakeRotate(boolean buttonPressed){
        if (buttonPressed && !outtakeRotated && (timer2.milliseconds() >= 100)) {
            outtakeRotate.setPosition(.25); //rotate outake in
            outtakeRotated = true;
            timer2.reset();
        } else if (buttonPressed && outtakeRotated && (timer2.milliseconds() >= 100)){
            outtakeRotate.setPosition(.75); //rotate outake out
            outtakeRotated = false;
            timer2.reset();
        }
    }

    public boolean outtakeSensorActive(){
        return outtakeSensor.isPressed();
    }
}