package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot_Configure.DEFAULT_LIFTING_ANGLE;
import static org.firstinspires.ftc.teamcode.Robot_Configure.DELAY;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShootingSystem implements Task{
    private DcMotor shootingMotor;
    private Servo liftingServo;
    private ElapsedTime runtime;
    private double shooting_time;
    private boolean motorOn = false;
    enum Status{
            IDLE,
        SHOOTING,
    }
    Status status = Status.IDLE;

    ShootingSystem(DcMotor shootingMotor, Servo liftingServo, ElapsedTime timer){
        this.shootingMotor = shootingMotor;
        this.liftingServo = liftingServo;
        runtime = timer;
    }
    public void init(){
        shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftingServo.setDirection(Servo.Direction.FORWARD);
    }
    // 발사 모터를 가동
    public void startMotor(double power) {
        motorOn = true;
        shootingMotor.setPower(power);
    }
    public void stopMotor(){
        motorOn = false;
        shootingMotor.setPower(0);
    }
    // 발사 코드
    public void shoot(){
        if(status != Status.IDLE) return;
        liftingServo.setPosition(DEFAULT_LIFTING_ANGLE / 300 );

        status = Status.SHOOTING;
        shooting_time = runtime.seconds();
    }
    // 상태 업데이트
    public void process(){
        switch (status){
            case SHOOTING:
                if(shooting_time + DELAY <= runtime.seconds()){
                    liftingServo.setPosition(0);
                    status = Status.IDLE;
                }
                break;
        }
    }
    public boolean isBusy(){
        return status != Status.IDLE;
    }
    public boolean isMotorOn() { return motorOn; }
}