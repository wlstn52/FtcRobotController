package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_ENCODER_PPR;
import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_RADIUS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Two Drive(Tank)", group="Test")
public class TwoDrive extends LinearOpMode {
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "ready to test");
        telemetry.update();

        waitForStart();
        runtime.reset();

        backRightDrive.setPower(0.8);
        backLeftDrive.setPower(0.8);

        int moveTicks = (int) ((7.87 * WHEEL_ENCODER_PPR) / (WHEEL_RADIUS * 2 * Math.PI));
        backRightDrive.setTargetPosition(moveTicks  + backLeftDrive.getCurrentPosition());
        backLeftDrive.setTargetPosition(moveTicks  + backRightDrive.getCurrentPosition());

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "test done");
        telemetry.update();
        sleep(20000);
    }

}
