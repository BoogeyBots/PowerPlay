package org.firstinspires.ftc.teamcode.test.vechi.nou;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "PE CICLU")
public class TestCiclu extends LinearOpMode {

    Brat brat = new Brat(hardwareMap);

    Servo servo_gheare = null;

    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;

    public Servo servoST = null;
    public Servo servoDR = null;


    @Override
    public void runOpMode() throws InterruptedException {
        servo_gheare = hardwareMap.get(Servo.class, "servo_gheara");
        servo_gheare.setPosition(0.0);
        motorDR_ENC = hardwareMap.get(DcMotorEx.class, "motorST");
        motorST = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorDR_ENC.setTargetPosition(0);
        motorDR_ENC.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorDR_ENC.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorST.setDirection(DcMotorEx.Direction.REVERSE);
        motorST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorST.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorDR_ENC.setVelocityPIDFCoefficients(16.0, 2.0, 0.0 , 5.0);
        motorDR_ENC.setPower(1.0);

        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");

        servoST.setPosition(0.5);
        servoDR.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {

                servoST.setPosition(0.1238);
                servoDR.setPosition(1.0 - 0.1238);
            } else if (gamepad1.b) {
                servoST.setPosition(0.608);
                servoDR.setPosition(1.0 - 0.608);

            }

            if (gamepad1.x){
                servo_gheare.setPosition(0.0);
            }
            else if(gamepad1.y){
                servo_gheare.setPosition(0.4);
            }

            if(gamepad1.right_bumper){
                motorDR_ENC.setTargetPosition(3170);
            }
            if (gamepad1.left_bumper){
                motorDR_ENC.setTargetPosition(0);
            }

            motorST.setVelocity(motorDR_ENC.getVelocity());


        }

    }

}
