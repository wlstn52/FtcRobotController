package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Autonomous(name="Color Sensor Test", group="Test")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.BLACK
                )
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .build();
        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n");

            // Request the most recent color analysis.  This will return the closest matching
            // colorSwatch and the predominant color in the RGB, HSV and YCrCb color spaces.
            // The color space values are returned as three-element int[] arrays as follows:
            //  RGB   Red 0-255, Green 0-255, Blue 0-255
            //  HSV   Hue 0-180, Saturation 0-255, Value 0-255
            //  YCrCb Luminance(Y) 0-255, Cr 0-255 (center 128), Cb 0-255 (center 128)
            //
            // Note: to take actions based on the detected color, simply use the colorSwatch or
            // color space value in a comparison or switch.   eg:

            //    if (result.closestSwatch == PredominantColorProcessor.Swatch.RED) {.. some code ..}
            //  or:
            //    if (result.RGB[0] > 128) {... some code  ...}

            PredominantColorProcessor.Result result = colorSensor.getAnalysis();

            // Display the Color Sensor result.
            telemetry.addData("Best Match", result.closestSwatch);
            telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                    result.RGB[0], result.RGB[1], result.RGB[2]));
            telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                    result.HSV[0], result.HSV[1], result.HSV[2]));
            telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)",
                    result.YCrCb[0], result.YCrCb[1], result.YCrCb[2]));
            telemetry.update();

            sleep(20);
        }
    }
}
