package org.firstinspires.ftc.teamcode;

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

        backRightDrive.setPower(1);
        backLeftDrive.setPower(1);

        sleep(1000 * 5);

        telemetry.addData("Status", "test done");
        telemetry.update();
        sleep(100);
    }

}
