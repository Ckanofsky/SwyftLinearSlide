package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    Robot robot;
    private DcMotorEx IntakeMotor;

    private TouchSensor intakeSensor;

    Intake(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        IntakeMotor = robot.hwMap.get(DcMotorEx.class, "intakeMotor");
        IntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSensor = robot.hwMap.get(TouchSensor.class, "intakeSensor");
    }

    public void intakeIn(){ IntakeMotor.setPower(1);}
    public void intakeOut(){ IntakeMotor.setPower(-1);}

    public void intakeManual(double trigger1, double trigger2){
        if (trigger1 >= .75){ intakeIn();}
        if (trigger2 >= .75){ intakeOut();}
    }
    public boolean intakeSensorActive(){
        return intakeSensor.isPressed();
    }
}