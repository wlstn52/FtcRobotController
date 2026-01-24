package org.firstinspires.ftc.teamcode;

import android.graphics.Path;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="Cam Traking Test", group = "Test")
public class CamTrakingTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private Servo camServo;
    private WebcamName webcam1;

    private static final int width = 640;
    private static final int height = 480;
    private static double MAX_ANGLE = 150.0;

    private double angle = 0;
    @Override
    public void runOpMode(){
        camServo = hardwareMap.get(Servo.class, "cam_servo");
        camServo.setDirection(Servo.Direction.FORWARD);

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilInit();
        waitForStart();
        while(opModeIsActive()){
            for(AprilTagDetection detection : aprilTagProcessor.getDetections()){
                if(detection.metadata != null){
                    double yaw = detection.robotPose.getOrientation().getYaw();
                    angle = -yaw;
                    camServo.setPosition(angle / MAX_ANGLE);
                }
            }
        }
    }
    private void aprilInit(){
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder().setCamera(webcam1)
                .setCameraResolution(new Size(width, height))
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor).build();
    }
}
