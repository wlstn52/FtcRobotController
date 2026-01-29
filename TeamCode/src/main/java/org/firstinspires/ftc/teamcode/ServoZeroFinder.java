package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Zero Point Finder", group="Utility")
public class ServoZeroFinder extends LinearOpMode {

    private Servo myServo;
    private double targetPosition = 0.5; // 시작은 중앙점

    @Override
    public void runOpMode() {
        // 하드웨어 맵에서 "test_servo"라는 이름의 서보를 가져옴
        myServo = hardwareMap.get(Servo.class, "test_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("A: 0.0 | B: 0.5 | X: 1.0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 버튼별 위치 제어
            if (gamepad1.a) {
                targetPosition = 0.0;
            } else if (gamepad1.b) {
                targetPosition = 0.5;
            } else if (gamepad1.x) {
                targetPosition = 1.0;
            }

            // 서보에 위치 명령 전달
            myServo.setPosition(targetPosition);

            // 현재 상태 화면 출력
            telemetry.addData("Servo Position", targetPosition);
            telemetry.addLine("Press A(0.0), B(0.5), X(1.0) to test");
            telemetry.update();
        }
    }
}
