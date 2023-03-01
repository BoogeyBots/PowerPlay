package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class GlisieraModule {
    HardwareMap hardwareMap;

    public GlisieraModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public static double kp=4.0, ki=0.0 , kd=0.1;
    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;
    PIDController controller = new PIDController(4.0, 0.0, 0.1);


   public void init() {

       motorDR_ENC = hardwareMap.get(DcMotorEx.class, "motorDR_ENC");
       motorST = hardwareMap.get(DcMotorEx.class, "motorST");

       motorDR_ENC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorST.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


       motorDR_ENC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motorST.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       motorDR_ENC.setDirection(DcMotorEx.Direction.REVERSE);

       controller.reset();

   }

    public void update(){
        if (!controller.atSetPoint()) {
            double output = controller.calculate(
                    motorST.getCurrentPosition()      // the measured value
            );
            motorST.setVelocity(output);
            motorDR_ENC.setVelocity(output);
        }
    }

    public void goUp(){
        controller.setSetPoint(1600);
    }

    public void goDown(){
       controller.setSetPoint(0);
    }

    public void goMid(){
      controller.setSetPoint(250);
    }



}




