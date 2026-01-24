package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Motor Test", group="Test")
public class testMotor extends LinearOpMode {
    private DcMotor testMoter = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        testMoter = hardwareMap.get(DcMotor.class, "left_drive");
        testMoter.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "ready to test");
        telemetry.update();

        waitForStart();

        testMoter.setPower(0.9);
        telemetry.addData("Status", "backward test");
        telemetry.update();
        runtime.reset();
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
