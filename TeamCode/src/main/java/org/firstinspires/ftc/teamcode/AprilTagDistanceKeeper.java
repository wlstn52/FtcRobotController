package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTag Distance Keeper", group = "Concept")
public class AprilTagDistanceKeeper extends LinearOpMode {

    // --- 설정값 ---
    final double TARGET_DISTANCE = 12.0; // 목표 거리 (단위: 인치)
    final double SPEED_GAIN  =  0.02;    // 거리 오차에 대한 속도 가중치 (P-Gain)
    final double STRAFE_GAIN =  0.015;   // 좌우 오차에 대한 가중치
    final double TURN_GAIN   =  0.01;    // 회전 오차에 대한 가중치
    final double MAX_AUTO_SPEED = 0.5;   // 최대 속도 제한


    private DcMotor leftDrive  = null;
    private DcMotor rightDrive = null;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // 모터 초기화

        leftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        // AprilTag 프로세서 생성
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = 0, strafe = 0, turn = 0;
            boolean targetFound = false;
            AprilTagDetection desiredTag = null;

            // 감지된 태그 리스트 가져오기
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    desiredTag = detection;
                    targetFound = true;
                    break; // 첫 번째로 발견된 태그 기준
                }
            }

            // --- 탱크 드라이브 전용 계산 부분 ---

            if (targetFound) {
                // 1. 오차 계산
                double rangeError = (desiredTag.ftcPose.range - TARGET_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;

                // 2. 가중치 적용 (P-제어)
                // drive: 앞뒤 이동량, turn: 좌우 회전량
                double drive = rangeError * SPEED_GAIN;
                double turn  = headingError * TURN_GAIN;

                // 3. 탱크 드라이브 공식 적용
                double leftPower  = drive + turn;
                double rightPower = drive - turn;

                // 4. 파워 정규화 (최대값 1.0 유지)
                double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
                if (max > 1.0) {
                    leftPower /= max;
                    rightPower /= max;
                }

                // 5. 모터에 적용
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);

            } else {
                // 태그를 찾지 못하면 정지
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            // 속도 제한 및 구동
            drive  = Math.max(Math.min(drive, MAX_AUTO_SPEED), -MAX_AUTO_SPEED);
            strafe = Math.max(Math.min(strafe, MAX_AUTO_SPEED), -MAX_AUTO_SPEED);
            turn   = Math.max(Math.min(turn, MAX_AUTO_SPEED), -MAX_AUTO_SPEED);

            moveRobot(drive, strafe, turn);
            telemetry.update();
        }
    }

    // 메카넘 휠 구동 함수
    public void moveRobot(double x, double y, double yaw) {
        double lb = x - y + yaw;
        double rb = x + y - yaw;
        double lf = x + y + yaw;
        double rf = x - y - yaw;

        double max = Math.max(Math.abs(lb), Math.max(Math.abs(rb), Math.max(Math.abs(lf), Math.abs(rf))));
        if (max > 1.0) {
            lb /= max; rb /= max; lf /= max; rf /= max;
        }

        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
    }
}
