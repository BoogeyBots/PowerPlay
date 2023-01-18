package org.firstinspires.ftc.teamcode.test.vechi.nou;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Glisiere PID", group = "Robot")
public class GlisierePID extends LinearOpMode {
    FtcDashboard dashboard;
    public static double kp=4.0, ki=2.0 , kd=0.0;
    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;
    int modifier = 50;
    int poz_max = 3170;
    int poz_min = 0;
    //pozitie mid=3400
    //pozite low=2600
    //pozitie high=
    //int maxPos = +2800;
    //int minPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorDR_ENC = hardwareMap.get(DcMotorEx.class, "motorDR_ENC");
        motorST = hardwareMap.get(DcMotorEx.class, "motorST");

        motorDR_ENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorST.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorDR_ENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorST.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorST.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDController controller = new PIDController(2.6 , 0.0 , 0.25);

        controller.reset();

        waitForStart();

        while (opModeIsActive()){


            if(gamepad1.dpad_up){
                modifier++;
            }
            else if(gamepad1.dpad_down){
                modifier--;
            }

            if(gamepad1.a){
                if(controller.getSetPoint()  >= poz_min && controller.getSetPoint() + modifier <= poz_max){
                    controller.setSetPoint(Range.clip(controller.getSetPoint() + modifier, poz_min, poz_max ));
                }
            }
            else if(gamepad1.b){
                if(controller.getSetPoint() - modifier >= poz_min && controller.getSetPoint() <= poz_max){
                    controller.setSetPoint(Range.clip(controller.getSetPoint() - modifier, poz_min, poz_max ));
                }
            }
            else
                controller.setSetPoint(controller.getSetPoint());


            controller.setP(kp);
            controller.setI(ki);
            controller.setD(kd);

                if(gamepad1.x){
                    controller.setSetPoint(3170);
                }
                if (gamepad1.y){
                controller.setSetPoint(0);
            }

            if (!controller.atSetPoint()) {
                double output = controller.calculate(
                        motorDR_ENC.getCurrentPosition()      // the measured value
                );
                motorST.setVelocity(output);
                motorDR_ENC.setVelocity(output);
            }


            dashboard.updateConfig();
            telemetry.addData("Pozitia curenta", motorDR_ENC.getCurrentPosition());
            telemetry.addData("Pozitia care trb atinsa", controller.getSetPoint());
            telemetry.update();
        }
    }

}