package org.firstinspires.ftc.teamcode.test.vechi.nou;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Two Servo", group = "Robot")
public class TestTwoServo1 extends LinearOpMode {
    public Servo servoST = null;
    public Servo servoDR = null;

    double resolution = 0.00005;
    double resChangeSpeed = 0.00000001;

    @Override
    public void runOpMode() throws InterruptedException {
        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");

        servoST.setPosition(0.5);
        servoDR.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                resolution += resChangeSpeed;
            }
        }
    }
}