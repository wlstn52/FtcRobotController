package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Test Cam", group = "Test")
public class testCam extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final int width = 640;
    private static final int height = 480;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    @Override
    public void runOpMode(){
        initApril();
        waitForStart();
        while(opModeIsActive()){
            telemetryAprilTag();
            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private void initApril(){
        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        builder.setCameraResolution(new Size(width, height));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }
}
