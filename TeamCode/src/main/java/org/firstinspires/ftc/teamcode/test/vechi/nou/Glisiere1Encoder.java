package org.firstinspires.ftc.teamcode.test.vechi.nou;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Glisiere 1 Encoder", group = "Robot")
public class Glisiere1Encoder extends LinearOpMode {



    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;
    int modifier = 300;
    int poz_max = 3170;
    int poz_min = 0;
    //pozitie mid=3400
    //pozite low=2600
    //pozitie high=
    //int maxPos = +2800;
    //int minPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        waitForStart();

        while (opModeIsActive()){
            /*
            if(gamepad1.a){
                if(motorDR_ENC.getTargetPosition()  >= poz_min && motorDR_ENC.getTargetPosition() + modifier <= poz_max){
                    motorDR_ENC.setTargetPosition(Range.clip(motorDR_ENC.getTargetPosition() + modifier, poz_min, poz_max ));
                }
            }
            else if(gamepad1.b){
                if(motorDR_ENC.getTargetPosition() - modifier >= poz_min && motorDR_ENC.getTargetPosition() <= poz_max){
                    motorDR_ENC.setTargetPosition(Range.clip(motorDR_ENC.getTargetPosition() - modifier, poz_min, poz_max ));
                }
            }
            else
                motorDR_ENC.setTargetPosition(motorDR_ENC.getCurrentPosition());
*/
            if(gamepad1.x){
                motorDR_ENC.setTargetPosition(3170);
            }
            if (gamepad1.y){
                motorDR_ENC.setTargetPosition(0);
            }

            if(gamepad1.dpad_up){
                modifier++;
            }
            else if(gamepad1.dpad_down){
                modifier--;
            }

            motorST.setVelocity(motorDR_ENC.getVelocity());


            telemetry.addData("Pozitia curenta", motorDR_ENC.getCurrentPosition());
            telemetry.addData("Modifier", modifier);
            telemetry.addData("Velocity motor DR", motorDR_ENC.getVelocity());
            telemetry.addData("Velocity motor ST", motorST.getVelocity());
            telemetry.update();
        }
    }

}