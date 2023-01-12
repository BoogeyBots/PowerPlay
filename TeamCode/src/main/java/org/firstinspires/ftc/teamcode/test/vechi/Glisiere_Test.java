package org.firstinspires.ftc.teamcode.test.vechi;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Glisiere", group = "Robot")
public class Glisiere_Test extends LinearOpMode {

    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;
    int modifier = 50;
    //pozitie mid=3400
    //pozite low=2600
    //pozitie high=
    //int maxPos = +2800;
    //int minPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motorDR_ENC = hardwareMap.get(DcMotorEx.class, "motorST");
        motorST = hardwareMap.get(DcMotorEx.class, "motorDR");
        motorDR_ENC.setTargetPosition(0);
        motorDR_ENC.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorDR_ENC.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        motorST.setTargetPosition(0);
        motorST.setDirection(DcMotorEx.Direction.REVERSE);
        motorST.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorDR_ENC.setVelocityPIDFCoefficients(3.0, 1.0, 0.0 , 0.0);
        motorDR_ENC.setPower(0.3);
        motorST.setVelocityPIDFCoefficients(3.0, 1.0, 0.0, 0.0);
        motorST.setPower(0.3);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a){
                motorDR_ENC.setTargetPosition(motorDR_ENC.getCurrentPosition() + modifier);
                motorST.setTargetPosition(motorST.getCurrentPosition() + modifier);
            }
            else if(gamepad1.b){
                motorDR_ENC.setTargetPosition(motorDR_ENC.getCurrentPosition() - modifier);
                motorST.setTargetPosition(motorST.getCurrentPosition() - modifier);

            }
            else
                motorDR_ENC.setTargetPosition(motorDR_ENC.getCurrentPosition());
                motorST.setTargetPosition(motorST.getCurrentPosition());

            if(gamepad1.dpad_up){
                modifier++;
            }
            else if(gamepad1.dpad_down){
                modifier--;
            }


            telemetry.addData("Pozitia curenta", motorDR_ENC.getCurrentPosition());
            telemetry.addData("Modifier", modifier);
            telemetry.addData("Velocity Motor DR", motorDR_ENC.getVelocity());
            telemetry.addData("Velocity Motor ST", motorST.getVelocity());

            telemetry.addData("Power Motor DR", motorDR_ENC.getPower());
            telemetry.update();
        }
    }

}