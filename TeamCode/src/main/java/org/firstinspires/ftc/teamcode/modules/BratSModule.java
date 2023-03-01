package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class   BratSModule {
    HardwareMap hardwareMap;

    public BratSModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Servo servoST = null;
    public Servo servoDR = null;


    public void init() {

        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");
        servoST.setPosition(0.105);//
        servoDR.setPosition(1.0 - 0.105);

    }


    public void goDown()
    {
        servoST.setPosition(0.105);
        servoDR.setPosition(1.0 - 0.105);
    }
    public void goUp()
    {
        servoST.setPosition(0.65);
        servoDR.setPosition(1.0 - 0.65);
    }
    public void goBlack() {
        servoST.setPosition(0.14);
        servoDR.setPosition(1.0 - 0.14);
    }
    public void autonom()
    {
        servoST.setPosition(0.183);
        servoDR.setPosition(1-0.183); //0.1738
    }
    public void goLow()
    {
        servoST.setPosition(0.8); //0.319444
        servoDR.setPosition(1-0.8 );
    }
    public void goSemi()
    {
        servoST.setPosition(0.5);
        servoDR.setPosition(1-0.5);
    }
    public void autonom2()
    {
        servoST.setPosition(0.1738);
        servoDR.setPosition(1 - 0.1738);
    }
    public void autonom3()
    {
        servoST.setPosition(0.15166);
        servoDR.setPosition(1 - 0.15166);
    }
    public void autonom4()
    {
        servoST.setPosition(0.1244444);
        servoDR.setPosition(1 - 0.1244444);
    }
    public void autonom5()
    {
        servoST.setPosition(0.1088888);
        servoDR.setPosition(1 - 0.1088888);
    }
}




