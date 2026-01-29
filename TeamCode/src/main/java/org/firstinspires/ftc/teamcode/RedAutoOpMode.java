package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot_Configure.SHOOTING_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_ENCODER_PPR;
import static org.firstinspires.ftc.teamcode.Robot_Configure.WHEEL_RADIUS;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

// RED
@Autonomous(name="RedAutoOpMode", group="Auto")
public class RedAutoOpMode extends LinearOpMode {
    final static int TEAM_ID = 24; // RED
    enum Status {
        IDLE,
        SORTING,
        SEARCHING,
        DRIVING,
        SHOOTING, ALIGNING,

    }
    Status status = Status.IDLE;
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

        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        //motorSystem = new MotorSystem(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        //initApril();

        shootingSystem = new ShootingSystem(shootingMotor, liftingServo, runtime);;
        intakeSystem = new IntakeSystem(intakeMotor, sortingServo);

        shootingSystem.init();
        intakeSystem.init();

        /*
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        sortingServo.setDirection(Servo.Direction.FORWARD);
        sortingServo.setDirection(Servo.Direction.FORWARD);

         */

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.enableLiveView(true);
        builder.addProcessor(colorSensor);

        visionPortal = builder.build();

        waitForStart();
        runtime.reset();
        intakeMotor.setPower(0.6);

        // SEARCHING
        for(int step = 0; step <= 30; step++) {
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                if (detection.metadata == null) continue;
                if (detection.id == 24 || detection.id == 20) continue;
                TagId = detection.id;
                status = Status.DRIVING;
                return;
            }
            webcamServo.setPosition(0.5 + (6 * step) / 300.0);
            sleep(100);
        }

        webcamServo.setPosition(0.5);
        sleep(500);

        double y = -1, distanceInches = 0; // inch
        for(AprilTagDetection detection : aprilTagProcessor.getDetections()){
            if (detection.metadata == null) continue;
            if(detection.id != TEAM_ID) continue;
            y = detection.ftcPose.y;
        }
        distanceInches = y;

        int moveTicks = (int) ((distanceInches * WHEEL_ENCODER_PPR) / (WHEEL_RADIUS * 2 * Math.PI));

        // DRIVING
        backLeftDrive.setTargetPosition(backLeftDrive.getCurrentPosition() + moveTicks);
        backRightDrive.setTargetPosition(backRightDrive.getCurrentPosition() + moveTicks);

        backLeftDrive.setPower(0.5);
        backRightDrive.setPower(0.5);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // SHOOTING & SORTING
        // 처음 가지고 있는 아티팩트 배치 PPG
        int[][] sorting_order = {
                {2, 1, 0}, // 21
                {2, 3, 1}, // 22
                {2, 1, 3} // 23
        };

        for(int i = 0; i < 3; i++){
            intakeSystem.setSorting(sorting_order[TagId-21][i]);
            sleep(1000);
            shootingSystem.startMotor(SHOOTING_MOTOR_POWER);
            sleep(1500);
            shootingSystem.shoot();
            shootingSystem.stopMotor();
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
}
