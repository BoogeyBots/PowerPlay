package org.firstinspires.ftc.teamcode.test.vechi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DCTest", group = "Robot")
public class DC_Test extends LinearOpMode {
        private DcMotor motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int modifier = 500;
        motor.setTargetPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                motor.setPower(1.0);
                motor.setTargetPosition(motor.getCurrentPosition() + modifier);
            }
            else {
                motor.setPower(0.0);
                motor.setTargetPosition(motor.getCurrentPosition());
            }
                telemetry.addData("Pozitia curenta", motor.getCurrentPosition());
            telemetry.addData("Viteza: ", motor.getPower());
            telemetry.update();
        }
    }
}
