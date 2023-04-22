package org.firstinspires.ftc.teamcode.modules;

import  com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class IntakeModule {
    HardwareMap hardwareMap;

    public IntakeModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    Servo servo_gheare = null;


    public void init() {

        servo_gheare = hardwareMap.get(Servo.class, "servo_gheara");
        servo_gheare.setPosition(0.0);

    }
    public void initializare()
    {
        servo_gheare = hardwareMap.get(Servo.class, "servo_gheara");
    }


    public void open()
    {
        servo_gheare.setPosition(0.0);
    }

    public void close()
    {
        servo_gheare.setPosition(0.50);
    }



}




