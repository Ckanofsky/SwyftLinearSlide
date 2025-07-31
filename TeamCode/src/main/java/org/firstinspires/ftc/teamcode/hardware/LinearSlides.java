package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LinearSlides {
    Robot robot;

    private final double slideSpeed = 0.3;
    private DcMotorEx leftLinearSlide;
    private DcMotorEx rightLinearSlide;

    private TouchSensor leftSlideSensor;
    private TouchSensor rightSlideSensor;

    LinearSlides(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        leftLinearSlide = robot.hwMap.get(DcMotorEx.class, "leftLinearSlide");
        leftLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideSensor = robot.hwMap.get(TouchSensor.class, "leftSlideSensor");

        rightLinearSlide = robot.hwMap.get(DcMotorEx.class, "rightLinearSlide");
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideSensor = robot.hwMap.get(TouchSensor.class, "rightSlideSensor");
    }

    public void setSlidePosition(int pos){
        leftLinearSlide.setTargetPosition(pos);
        rightLinearSlide.setTargetPosition(pos);
        leftLinearSlide.setPower(slideSpeed);
        rightLinearSlide.setPower(slideSpeed);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideReset();
    }

    public int getSlidePosition(){
        return Math.max(leftLinearSlide.getCurrentPosition(), rightLinearSlide.getCurrentPosition());
    }

    private boolean limitReached = false;
    ElapsedTime resetTimer = new ElapsedTime();
    public void slideReset(){
        if (slideSensorActive() && !limitReached) {
            resetTimer.reset();                                     //Reset Timer
            limitReached = true;                                    //Limit Reached prevents reseting continously
            leftLinearSlide.setTargetPosition(-10000);                       //Large # to go down
            rightLinearSlide.setTargetPosition(-10000);
            leftLinearSlide.setPower(.2);                                    //Press against the bottom
            rightLinearSlide.setPower(.2);
            leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);        //Tune power in .setPower for more accuracy
            rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (resetTimer.milliseconds() >= 500 && slideSensorActive() && limitReached){//TimeDelay for reset + double check
            leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Reset the encoders
            rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            limitReached = false;                                   //Prevent looping
        }
    }


    int rateOfPositionIncrease = 1; //LinearSlideConstant
    public void manualLinearSlide(boolean extend, boolean retract){
        if (extend) {
            setSlidePosition(getSlidePosition() + rateOfPositionIncrease);
        } else if (retract){
            setSlidePosition(getSlidePosition() - rateOfPositionIncrease);
        }
    }

    public boolean slideSensorActive(){
        return leftSlideSensor.isPressed() && rightSlideSensor.isPressed();
    }



}