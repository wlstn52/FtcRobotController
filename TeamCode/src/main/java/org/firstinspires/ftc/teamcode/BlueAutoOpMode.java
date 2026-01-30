package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ArenaConst.ARENA_EDGE;
import static org.firstinspires.ftc.teamcode.Robot_Configure.DELAY;
import static org.firstinspires.ftc.teamcode.Robot_Configure.SHOOTING_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.Robot_Configure.SHOOT_DISTANCE;
import static org.firstinspires.ftc.teamcode.Robot_Configure.TURN_GAIN;
import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_ENCODER_PPR;
import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_RADIUS;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.math.MathContext;

// BLUE
@Autonomous(name="BlueAutoOpMode", group="Auto")
public class BlueAutoOpMode extends LinearOpMode {
    final static int TEAM_ID = 20; // BLUE
    private ElapsedTime runtime = new ElapsedTime();
//    private MotorSystem motorSystem;

    // Hardware - DC Motors
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shootingMotor = null;
    private DcMotor intakeMotor = null;

    private Servo sortingServo;
    private Servo liftingServo;
    private Servo webcamServo;

    private ShootingSystem shootingSystem;
    private IntakeSystem intakeSystem;

    // Hardware - Cameras

    private PredominantColorProcessor colorSensor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private WebcamName webcam;

    private IMU imu;

    private double time = 0;
    private boolean isWaiting = false;
    private double delay = 0;
    private int TagId = 21;
/*
    private void initApril(){
        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.enableLiveView(true);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        //visionPortal.resumeStreaming();
    }
*/
    @Override
    public void runOpMode(){
        // Hardware embedding
        //frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        //frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        shootingMotor = hardwareMap.get(DcMotor.class, "shooting_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        sortingServo = hardwareMap.get(Servo.class, "sorting_servo");
        liftingServo = hardwareMap.get(Servo.class, "lifting_servo");
        webcamServo = hardwareMap.get(Servo.class, "webcam_servo");
        imu = hardwareMap.get(IMU.class, "imu");

        // Control Hub 장착 방향 설정
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

// IMU 초기화
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        webcamServo.setDirection(Servo.Direction.REVERSE);

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //motorSystem = new MotorSystem(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        //initApril();

        shootingSystem = new ShootingSystem(shootingMotor, liftingServo, runtime);;
        intakeSystem = new IntakeSystem(intakeMotor, sortingServo);

        shootingSystem.init();
        intakeSystem.init();

        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shootingMotor.setDirection(DcMotor.Direction.FORWARD);
        sortingServo.setDirection(Servo.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        visionInit();
        waitForStart();
        runtime.reset();

        // Scan Obelisk
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata == null) continue;
            if (detection.id == 24 || detection.id == 20) continue;
            TagId = detection.id;
            break;
        }

        // DRIVING
        drive_distance(ARENA_EDGE, ARENA_EDGE, 0.7, 0.7);

        turn(-45);

        double d = ARENA_EDGE * Math.sqrt(0.5) - SHOOT_DISTANCE;
        drive_distance(d, d, 0.7, 0.7);

        // SHOOTING & SORTING
        // 처음 가지고 있는 아티팩트 배치 PPG
        int[][] sorting_order = {
                {2, 1, 0}, // 21
                {2, 3, 1}, // 22
                {2, 1, 3} // 23
        };

        shootingSystem.startMotor(SHOOTING_MOTOR_POWER);
        sleep(1500);
        for(int i = 0; i < 3; i++){
            intakeSystem.setSorting(sorting_order[TagId-21][i]);
            shootingSystem.shoot();
            sleep((long) DELAY * 1000);
        }
        shootingSystem.stopMotor();

        // ESCAPE
        drive_distance(-d, -d, 0.7, 0.7);
        turn(45);
        drive_distance(-10, -10, -0.7, -0.7);
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
    public void drive_distance(double l_dist, double r_dist, double lpower, double rpower){
        int lmoveticks = (int) ((l_dist * WHEEL_ENCODER_PPR) / (2 * WHEEL_RADIUS * Math.PI));
        int rmoveticks = (int) ((r_dist * WHEEL_ENCODER_PPR) / (2 * WHEEL_RADIUS * Math.PI));

        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + lmoveticks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + rmoveticks);

        backLeftDrive.setPower(lpower);
        backRightDrive.setPower(rpower);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turn(double targetAngle){
        double error = targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while (opModeIsActive() && Math.abs(error) > 1.0) {
            // 현재 각도 업데이트
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = targetAngle - currentAngle;

            double motorPower = error * TURN_GAIN;

            double minPower = 0.15;
            if (motorPower > 0 && motorPower < minPower) motorPower = minPower;
            if (motorPower < 0 && motorPower > -minPower) motorPower = -minPower;

            // 4. 모터에 힘 전달 (왼쪽은 뒤로, 오른쪽은 앞으로 - 제자리 회전)
            backLeftDrive.setPower(-motorPower);
            backRightDrive.setPower(motorPower);

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Current", currentAngle);
            telemetry.update();
        }

        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
