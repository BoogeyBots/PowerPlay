package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Glisiere", group = "Robot")
public class Glisiere_Test extends LinearOpMode {

    public DcMotorEx motor = null;
    int modifier = 100;
    //pozitie mid=3400
    //pozite low=2600
    //pozitie high=
    //int maxPos = +2800;
    //int minPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "glisiere");
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocityPIDFCoefficients(15.0, 3.0, 0.0 , 0.0);
        motor.setPower(0.9);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                motor.setTargetPosition(motor.getCurrentPosition() + modifier);
            }
            else if(gamepad1.b){
                motor.setTargetPosition(motor.getCurrentPosition() - modifier);
            }
            else
                motor.setTargetPosition(motor.getCurrentPosition());
            if(gamepad1.dpad_up){
                modifier++;
            }
            else if(gamepad1.dpad_down){
                modifier--;
            }

            telemetry.addData("Pozitia curenta", motor.getCurrentPosition());
            telemetry.addData("Modifier", modifier);
            telemetry.update();
        }
    }

}