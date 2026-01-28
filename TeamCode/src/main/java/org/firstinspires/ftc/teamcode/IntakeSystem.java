package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
public class IntakeSystem {
    private Servo sortingServo;
    private DcMotor intakeMotor;
    private boolean motorOn = false;
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    public enum BallType{
        EMPTY,
        Purple,
        GREEN
    };
    public int front = 0;
    IntakeSystem(DcMotor intake_motor, Servo servo){
        intakeMotor = intake_motor;
        sortingServo = servo;
    }
    public void init(){
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    public int getFront(){
        return front;
    }
    public void startMotor(double power){
        motorOn = true;
        intakeMotor.setPower(power);
    }
    public void stopMotor(){
        motorOn = false;
        intakeMotor.setPower(0);
    }

    public boolean isMotorOn() { return motorOn; }
    public void revolveSorting(Servo.Direction direction) {
        switch(direction){
            case FORWARD:
                front++;
            case REVERSE:
                front--;
        }
        if(front >= 3) front = 2;
        if(front < 0) front = 0;

        sortingServo.setPosition(120.0 / 300 * front);
    }

}

