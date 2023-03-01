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
    public static double kp=4.0, ki=0.0 , kd=0.1;
    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;
    int modifier = 50;
    int poz_max = 1600;
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

        motorST.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDR_ENC.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDController controller = new PIDController(4.0 , 0.0 , 0.1);

        controller.reset();

        // controller.setTolerance(0.1);

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
            else {
                controller.setSetPoint(controller.getSetPoint());
            }

            controller.setP(kp);
            controller.setI(ki);
            controller.setD(kd);

            if(gamepad1.x){
                controller.setSetPoint(1600);
            }
            if (gamepad1.y){
                controller.setSetPoint(0);
            }

            if (Math.abs(motorST.getCurrentPosition() - controller.getSetPoint()) < 2.0) {
                double output = controller.calculate(
                        motorST.getCurrentPosition()      // the measured value
                );
                telemetry.addData("Output", output);
                motorST.setVelocity(output);
                motorDR_ENC.setVelocity(output);
            }


            dashboard.updateConfig();
            telemetry.addData("A AJUNS?", controller.atSetPoint());
            telemetry.addData("Pozitia curenta", motorST.getCurrentPosition());
            telemetry.addData("Pozitia care trb atinsa", controller.getSetPoint());
            telemetry.addData("Eroare Pozitie", controller.getPositionError());
            telemetry.update();
        }
    }

}