package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Autonomous(name="AutoOpMode", group="Auto")
public class AutoOpMode extends OpMode {
    enum Status {
        IDLE,
        SORTING,
        DRIVING,
        SHOOTING,

    }
    Status status = Status.IDLE;
    private ElapsedTime runtime = new ElapsedTime();
//    private MotorSystem motorSystem;

    // Hardware - DC Motors
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shootingMotor = null;
    private DcMotor intakeMotor = null;

    private Servo sortingServo;
    private Servo liftingServo;

    // Hardware - Cameras

    private PredominantColorProcessor colorSensor;
    private VisionPortal visionPortal;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private double time = 0;
    private boolean isWaiting = false;
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
    public void init(){
        // Hardware embedding
        //frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        //frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        shootingMotor = hardwareMap.get(DcMotor.class, "shooting_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        sortingServo = hardwareMap.get(Servo.class, "sorting_servo");
        liftingServo = hardwareMap.get(Servo.class, "lifting");

        //motorSystem = new MotorSystem(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        //initApril();

        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootingMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        sortingServo.setDirection(Servo.Direction.FORWARD);
        sortingServo.setDirection(Servo.Direction.FORWARD);

        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK
                )
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.enableLiveView(true);
        builder.addProcessor(colorSensor);

        visionPortal = builder.build();

    }

    @Override
    public void start(){
        runtime.reset();
        intakeMotor.setPower(0.6);
    }

    @Override
    public void loop(){
        switch (status){
            case IDLE:
                telemetry.addData("Status", "IDLE");
                PredominantColorProcessor.Result result = colorSensor.getAnalysis();
                if(result.closestSwatch != PredominantColorProcessor.Swatch.BLACK){
                    status = Status.SORTING;
                }
                break;

            case SORTING:
                telemetry.addData("Status", "SORTING");
                sortingServo.setPosition(240.0/300);
                status = Status.DRIVING;
                break;

            case DRIVING:
                telemetry.addData("Status", "DRIVING");
                if(!isWaiting) {
                    backLeftDrive.setPower(0.7);
                    backRightDrive.setPower(0.7);
                    isWaiting = true;
                    time = runtime.time();
                }
                if (isWaiting && runtime.time() >= time + 1000 * 5){
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    isWaiting = false;
                    status = Status.SHOOTING;
                }
                break;

            case SHOOTING:
                telemetry.addData("Status", "SHOOTING");
                if(!isWaiting) {
                    shootingMotor.setPower(0.9);
                    isWaiting = true;
                    time = runtime.time();
                    liftingServo.setPosition(10.0/300); // 각도 수정해야함
                }
                if (isWaiting && runtime.time() >= time + 1000 * 5){
                    shootingMotor.setPower(0);
                    isWaiting = false;
                    status = Status.SHOOTING;
                }
                break;
        }
        telemetry.update();
    }

    @Override
    public void stop(){
        visionPortal.close();
    }
}
