package org.firstinspires.ftc.teamcode.test.vechi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motoare_test", group = "Robot")
public class Motoare_test extends LinearOpMode {
    private DcMotor rightMotor = null;
    private DcMotor leftMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.get(DcMotor.class, "motorDR");
        leftMotor = hardwareMap.get(DcMotor.class, "motorST");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int modifier = 500;
        rightMotor.setTargetPosition(0);
        leftMotor.setTargetPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                rightMotor.setPower(1.0);
                leftMotor.setPower(1.0);

                rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + modifier);
                leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + modifier);
            }
            if (gamepad1.dpad_down){
                rightMotor.setPower(-1.0);
                leftMotor.setPower(-1.0);
            }
            else {
                rightMotor.setPower(0.0);
                leftMotor.setPower(0.0);

                rightMotor.setTargetPosition(rightMotor.getCurrentPosition());
                leftMotor.setTargetPosition(leftMotor.getCurrentPosition());
            }
            telemetry.addData("Pozitia curenta dreapta: ", rightMotor.getCurrentPosition());
            telemetry.addData("Pozitie curenta stanga: ", leftMotor.getCurrentPosition());

            telemetry.addData("Viteza dreapta: ", rightMotor.getPower());
            telemetry.addData("Viteza stanga: " , leftMotor.getPower());

            telemetry.update();
        }
    }
}
