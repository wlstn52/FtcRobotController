package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Shooting Test", group="Test")
public class testShooting extends LinearOpMode {
    private DcMotor testMoter = null;
    private Servo liftingServo;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        testMoter = hardwareMap.get(DcMotor.class, "shooting_motor");
        liftingServo = hardwareMap.get(Servo.class, "lifting_servo");
        testMoter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMoter.setDirection(DcMotor.Direction.FORWARD);
        liftingServo.setDirection(Servo.Direction.FORWARD);
        liftingServo.setPosition(0);

        telemetry.addData("Status", "ready to test");
        telemetry.update();

        waitForStart();
        liftingServo.setPosition(0.1);
        while(opModeIsActive() && (runtime.seconds() < 5.0)){
            telemetry.addData("Time", "%4.1f s Elapsed", runtime.seconds());
            telemetry.update();
        }
        testMoter.setPower(0);
        telemetry.addData("Status", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
