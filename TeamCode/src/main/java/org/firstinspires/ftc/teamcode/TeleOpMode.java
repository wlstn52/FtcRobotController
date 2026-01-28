package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot_Configure.INTAKE_MOTOR_POWER;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="TeleOp Mode", group = "TeleOp")
public class TeleOpMode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shootingMotor = null;
    private DcMotor intakeMotor = null;

    private Servo liftingServo = null;
    private Servo sortingServo = null;

    private ShootingSystem shootingSystem;
    private IntakeSystem intakeSystem;

    // 비전 관련
    private WebcamName webcam;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void init(){
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        shootingMotor = hardwareMap.get(DcMotor.class, "shooting_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        liftingServo = hardwareMap.get(Servo.class, "lifting_servo");
        sortingServo = hardwareMap.get(Servo.class, "sorting_servo");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        shootingSystem = new ShootingSystem(shootingMotor, liftingServo, runtime);
        shootingSystem.init();

        intakeSystem = new IntakeSystem(intakeMotor, sortingServo);
        intakeSystem.init();
    }

    /**
     * This method will be called repeatedly during the period between when
     * the INIT button is pressed and when the START button is pressed (or the
     * OpMode is stopped).
     */
    @Override
    public void init_loop() {
    }

    /**
     * This method will be called once, when the START button is pressed.
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /**
     * This method will be called repeatedly during the period between when
     * the START button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        telemetry_message();
        gamepad_control();
        shootingSystem.process();
    }

    /**
     * This method will be called once, when this OpMode is stopped.
     * <p>
     * Your ability to control hardware from this method will be limited.
     */
    @Override
    public void stop() {
        shootingSystem.stopMotor();
        intakeSystem.stopMotor();
        telemetry.addLine("Robot stopped");
        telemetry.update();
    }

    // 비전 관련 초기 설정
    private void visionInit(){
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor).build();
    }

    void gamepad_control(){
        double axial1   =  -gamepad1.left_stick_y;
        double axial2   =  -gamepad1.right_stick_y;

        backLeftDrive.setPower(axial1);
        backRightDrive.setPower(axial2);
        if(gamepad1.aWasPressed() && shootingSystem.isBusy()){
            shootingSystem.shoot();
        }
        if(gamepad1.leftBumperWasPressed() && shootingSystem.isBusy()) {
            intakeSystem.revolveSorting(Servo.Direction.FORWARD);
        }
        if(gamepad1.rightBumperWasPressed() && shootingSystem.isBusy()) {
            intakeSystem.revolveSorting(Servo.Direction.REVERSE);
        }
        if(gamepad1.xWasPressed()){
            // 정렬 모터 turn on / turn off
            if(intakeSystem.isMotorOn()){
                intakeSystem.startMotor(INTAKE_MOTOR_POWER);
            }else{
                intakeSystem.stopMotor();
            }
        }
        if(gamepad1.yWasPressed()){
            // 발사 모터 turn on / turn off
            if(shootingSystem.isMotorOn()){
                shootingSystem.startMotor(INTAKE_MOTOR_POWER);
            }else{
                shootingSystem.stopMotor();
            }
        }
    }
    // drive hub 메세지
    void telemetry_message(){
        String shootingMotorStatus = shootingSystem.isMotorOn() ? "ON" : "OFF";
        String intakingMotorStatus = intakeSystem.isMotorOn() ? "ON" : "OFF";
        // 작동 시간
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addLine("=System Status=");
        telemetry.addData("Motor Power", String.format("Left: %4.2f Right: %4.2f", (float) backLeftDrive.getPower(), (float) backRightDrive.getPower()));
        telemetry.addData("Shooting System", shootingSystem.status);
        telemetry.addData("Shooting Motor", shootingMotorStatus);
        telemetry.addData("Intaking System", shootingSystem.status);
        telemetry.addData("Intaking Motor", intakingMotorStatus);
        telemetry.addData("Front Slot", intakeSystem.getFront());

        telemetry.update();
    }

    void getMovement(){
        for(AprilTagDetection detection : aprilTagProcessor.getDetections()){
            if(detection.metadata == null) continue;
        }
    }
}
