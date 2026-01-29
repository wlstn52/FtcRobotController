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

    // index [0, 2]
    public void setSorting(int index) {
        front = index * 2;
        sortingServo.setPosition((120 * index) / 300.0);
    }

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
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        sortingServo.setPosition(0.0);
    }
    public int getFront(){
        return front;
    }
    public boolean canShoot(){
        return front % 2 == 0;
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
    public void revolveSorting(int direction) {
        front += direction;
        if(front > 5) front = 5;
        if(front < 0) front = 0;

        sortingServo.setPosition(60.0 / 300 * front);
    }

}

