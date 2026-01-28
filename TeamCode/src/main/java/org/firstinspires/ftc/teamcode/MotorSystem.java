package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorSystem {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    MotorSystem(DcMotor fL, DcMotor fR, DcMotor bL, DcMotor bR){
        frontLeft = fL;
        frontRight = fR;
        backLeft = bL;
        backRight = bR;
    }
    public void setPower(float fL, float fR, float bL, float bR) {
        frontLeft.setPower(fL);
        frontRight.setPower(fR);
        backRight.setPower(bL);
        backLeft.setPower(bR);
    }
    public void setDirection(
            DcMotorSimple.Direction fL,
            DcMotorSimple.Direction fR,
            DcMotorSimple.Direction bL,
            DcMotorSimple.Direction bR
    ){
        frontLeft.setDirection(fL);
        frontRight.setDirection(fR);
        backRight.setDirection(bL);
        backLeft.setDirection(bR);
    }
    public void setMotorMode(DcMotor.RunMode mode){
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void move(float dist, float angle){
    }
}
