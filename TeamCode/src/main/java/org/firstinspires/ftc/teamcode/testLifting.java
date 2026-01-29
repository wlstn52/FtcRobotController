package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Lifting Test", group="Test")
public class testLifting extends LinearOpMode {
    private Servo testMoter = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        testMoter = hardwareMap.get(Servo.class, "lifting_servo");
        testMoter.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "ready to test");
        telemetry.update();
        waitForStart();
        telemetry.addLine(String.format("%f",
                testMoter.getPosition()));
        telemetry.update();
        sleep(2000);
        testMoter.setPosition(0);
        telemetry.addLine(String.format("%f",
                testMoter.getPosition()));
        telemetry.update();
        sleep(1000);

    }
}
