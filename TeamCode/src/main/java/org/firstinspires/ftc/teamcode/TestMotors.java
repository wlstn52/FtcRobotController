package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Driver Station 화면에서 선택할 이름
@TeleOp(name = "Test: Run Motors 0 and 1", group = "Test")
public class TestMotors extends LinearOpMode {

    // 사용할 모터 변수 2개 선언
    private DcMotor motor0;
    private DcMotor motor1;

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------------
        // 1. 하드웨어 맵핑 (Hardware Mapping)
        // 코드의 변수와 실제 Control Hub의 포트를 연결하는 아주 중요한 단계입니다.
        // 따옴표 안의 이름("motor_0", "motor_1")은 설정(Config)과 똑같아야 합니다.
        // ------------------------------------------------------------------
        motor0 = hardwareMap.get(DcMotor.class, "motor_0"); // Port 0번 연결
        motor1 = hardwareMap.get(DcMotor.class, "motor_1"); // Port 1번 연결

        // (선택 사항) 모터 방향 설정
        // 만약 한쪽이 반대로 돈다면 FORWARD를 REVERSE로 바꾸세요.
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor1.setDirection(DcMotor.Direction.FORWARD);

        // ------------------------------------------------------------------
        // 2. 대기 상태
        // ------------------------------------------------------------------
        telemetry.addData("Status", "Ready to Start");
        telemetry.update();

        // 사용자가 '재생(▶)' 버튼을 누를 때까지 여기서 기다립니다.
        waitForStart();

        // ------------------------------------------------------------------
        // 3. 실행 상태 (반복문)
        // ------------------------------------------------------------------
        // '정지(■)' 버튼을 누르기 전까지 이 안의 코드가 계속 반복됩니다.
        while (opModeIsActive()) {

            // 모터 0번과 1번에 전력 공급 (범위: -1.0 ~ 1.0)
            // 여기서는 안전하게 0.5 (50% 파워)로 설정했습니다.
            motor0.setPower(0.5);
            motor1.setPower(0.5);

            // 현재 상태를 Driver Hub 화면에 표시
            telemetry.addData("Motor 0 Power", motor0.getPower());
            telemetry.addData("Motor 1 Power", motor1.getPower());
            telemetry.addData("Status", "Running...");
            telemetry.update();
        }
    }
}