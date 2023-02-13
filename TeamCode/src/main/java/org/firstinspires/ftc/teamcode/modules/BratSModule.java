package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class BratSModule {
    HardwareMap hardwareMap;

    public BratSModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Servo servoST = null;
    public Servo servoDR = null;


    public void init() {

        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");
        servoST.setPosition(0.12);
        servoDR.setPosition(1.0 - 0.12);

    }


    public void goDown()
    {
        servoST.setPosition(0.12);
        servoDR.setPosition(1.0 - 0.12);
    }

    public void goUp()
    {
        servoST.setPosition(0.625);
        servoDR.setPosition(1.0 - 0.625);
    }
    public void goBlack() {
        servoST.setPosition(0.175);
        servoDR.setPosition(1.0 - 0.175);
    }
    public void autonom()
    {
        servoST.setPosition(0.201666);
        servoDR.setPosition(1-0.201666);
    }


}




