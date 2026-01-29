package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot_Configure.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.Robot_Configure.SHOOTING_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.Robot_Configure.SHOOT_DISTANCE;
import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_ENCODER_PPR;
import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_RADIUS;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="TeleOp Mode", group = "TeleOp")
public class TeleOpMode extends OpMode {
    enum Status {
        MANUAL,
        AUTO
    }

    Status status;
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
    private GamepadInput input;

    private double dist;

    @Override
    public void init() {
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        shootingMotor = hardwareMap.get(DcMotor.class, "shooting_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        liftingServo = hardwareMap.get(Servo.class, "lifting_servo");
        sortingServo = hardwareMap.get(Servo.class, "sorting_servo");
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // 엔코더 초기화
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootingSystem = new ShootingSystem(shootingMotor, liftingServo, runtime);
        shootingSystem.init();

        intakeSystem = new IntakeSystem(intakeMotor, sortingServo);
        intakeSystem.init();

        input = new GamepadInput(gamepad1);

        visionInit();
        status = Status.MANUAL;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        input.update();
        gamepad_control();
        shootingSystem.process();

        // AUTO 모드(자동 이동) 중 도달 확인
        if (status == Status.AUTO) {
            if (!backLeftDrive.isBusy() && !backRightDrive.isBusy()) {
                switchToManual();
            }
        }

        telemetry_message();
    }

    @Override
    public void stop() {
        shootingSystem.stopMotor();
        intakeSystem.stopMotor();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void visionInit() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    void gamepad_control() {
        // 주행 제어 (MANUAL 상태일 때만 스틱 작동)
        if (status == Status.MANUAL) {
            double axial1 = -gamepad1.left_stick_y;
            double axial2 = -gamepad1.right_stick_y;
            backLeftDrive.setPower(axial1);
            backRightDrive.setPower(axial2);
        }

        // 슈팅 및 인테이크 제어
        if (input.aPressed() && !shootingSystem.isBusy()) {
            shootingSystem.shoot();
        }
        if (input.lbumperPressed() && !shootingSystem.isBusy()) {
            intakeSystem.revolveSorting(1);
        }
        if (input.rbumperPressed() && !shootingSystem.isBusy()) {
            intakeSystem.revolveSorting(-1);
        }
        if (input.xPressed()) {
            if (!intakeSystem.isMotorOn()) intakeSystem.startMotor(INTAKE_MOTOR_POWER);
            else intakeSystem.stopMotor();
        }
        if (input.yPressed()) {
            if (!shootingSystem.isMotorOn()) shootingSystem.startMotor(SHOOTING_MOTOR_POWER);
            else shootingSystem.stopMotor();
        }

        // B 버튼: 자동 정렬 시작 또는 취소
        if (input.bPressed()) {
            if (status == Status.AUTO) {
                // 이동 중 B를 누르면 즉시 중단
                switchToManual();
            } else {
                // 정지 상태에서 B를 누르면 AprilTag 찾아서 이동 시작
                dist = getMovement();
                if (dist != -1000.0) {
                    startAutoDrive(dist);
                }
            }
        }
    }

    // 자동 주행 시작 설정
    private void startAutoDrive(double distanceInches) {
        status = Status.AUTO;

        // 이동 거리를 틱 단위로 변환
        int moveTicks = (int) ((distanceInches * WHEEL_ENCODER_PPR) / (WHEEL_RADIUS * 2 * Math.PI));

        // 타겟 위치 설정 (현재 위치 + 이동 거리)
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + moveTicks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + moveTicks);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // 속도 설정 (0.0 ~ 1.0)
        backLeftDrive.setPower(0.6);
        backRightDrive.setPower(0.6);
    }

    // 수동 모드로 복귀 및 모터 정지
    private void switchToManual() {
        status = Status.MANUAL;
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void telemetry_message() {
        String shootingMotorStatus = shootingSystem.isMotorOn() ? "ON" : "OFF";
        String intakingMotorStatus = intakeSystem.isMotorOn() ? "ON" : "OFF";

        telemetry.addData("Status", status);
        telemetry.addLine("=System Status=");
        telemetry.addData("Drive Power", "L: %.2f, R: %.2f", backLeftDrive.getPower(), backRightDrive.getPower());
        telemetry.addData("Target Dist", dist == -1000.0 ? "None" : String.format("%.2f inch", dist));
        telemetry.addData("Shooting System", shootingSystem.status);
        telemetry.addData("Shooting Motor", shootingMotorStatus);
        telemetry.addData("Intaking Motor", intakingMotorStatus);
        telemetry.update();
    }

    double getMovement() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // ID 24 또는 20 탐지
                if (detection.id == 24 || detection.id == 20) {
                    // ftcPose.y는 로봇 정면 방향의 거리입니다.
                    double d = detection.ftcPose.y - SHOOT_DISTANCE;
                    return Math.sqrt(d*d - (0.52*39.37) * (0.52*39.37));
                }
            }
        }
        return -1000.0;
    }
}

/**
 * Gamepad 입력 감지 클래스 (엣지 디텍션)
 */
class GamepadInput {
    private Gamepad gamepad;
    private int x = 0, y = 0, a = 0, b = 0;
    private int left_bumper, right_bumper;

    GamepadInput(Gamepad pad) {
        gamepad = pad;
    }

    public void update() {
        x = gamepad.x ? x + 1 : 0;
        y = gamepad.y ? y + 1 : 0;
        a = gamepad.a ? a + 1 : 0;
        b = gamepad.b ? b + 1 : 0;
        left_bumper = gamepad.left_bumper ? left_bumper + 1 : 0;
        right_bumper = gamepad.right_bumper ? right_bumper + 1 : 0;
    }

    public boolean xPressed() { return x == 1; }
    public boolean yPressed() { return y == 1; }
    public boolean aPressed() { return a == 1; }
    public boolean bPressed() { return b == 1; }
    public boolean lbumperPressed() { return left_bumper == 1; }
    public boolean rbumperPressed() { return right_bumper == 1; }
}