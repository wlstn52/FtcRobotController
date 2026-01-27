package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Shooting Test", group="Test")
public class testShooting extends LinearOpMode {
    private DcMotor testMotor = null;
    private Servo liftingServo;
    private ShootingSystem shootingSystem;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        testMotor = hardwareMap.get(DcMotor.class, "shooting_motor");
        liftingServo = hardwareMap.get(Servo.class, "lifting_servo");
        runtime.reset();
        shootingSystem = new ShootingSystem(testMotor, liftingServo, runtime);
        shootingSystem.init();
        waitForStart();
        shootingSystem.motorStart(0.8);
        sleep(2000);
        shootingSystem.shoot();
        double a = runtime.seconds();
        while(a + 3 >= runtime.seconds()){
            shootingSystem.process();
        }
        shootingSystem.motorStop();
        sleep(1000);
    }
}
