package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Glisiere  {
    HardwareMap hardwareMap;

    public Glisiere(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public DcMotorEx motor = null;
    int modifier = 200;
    int poz_max = -3200;
    int poz_min = 0;
    //pozitie mid=3400
    //pozite low=2600
    //pozitie high=
    //int maxPos = +2800;

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "glisiere");
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocityPIDFCoefficients(15.0, 3.0, 0.0 , 0.0);
        motor.setPower(0.9);
    }

    public void goUp(){
        if(motor.getTargetPosition() >= poz_max && motor.getTargetPosition() <= poz_min){
            motor.setTargetPosition(Range.clip(motor.getTargetPosition() - modifier, poz_max, poz_min ));
        }
    }


    public void goDown(){
        if(motor.getTargetPosition() >= poz_max && motor.getTargetPosition() <= poz_min){
            motor.setTargetPosition(Range.clip(motor.getTargetPosition() + modifier, poz_max, poz_min));
        }
    }

    public void setPosition(int i) {
        motor.setTargetPosition(i);
    }
}
    //int minPos = 0;

