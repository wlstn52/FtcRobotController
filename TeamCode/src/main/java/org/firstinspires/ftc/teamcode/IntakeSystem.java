package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
public class IntakeSystem {
    private Servo sortingServo;
    private double pos = 0;
    private int direction = 0;
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    public enum BallType{
        EMPTY,
        Purple,
        GREEN
    };

    public BallType[] drum = {BallType.EMPTY, BallType.EMPTY, BallType.EMPTY};
    public int front = 0;
    IntakeSystem(Servo servo){
        sortingServo = servo;
    }
    public void init(){

    }
    public void intake(int color, boolean clockwise){
        if(clockwise){

        }
    }
}
