package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OdometrieModule {
    HardwareMap hardwareMap;

    public OdometrieModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    Servo servo_odometrie = null;


    public void init() {
        servo_odometrie = hardwareMap.get(Servo.class, "servo_odometrie");
        servo_odometrie.setPosition(0.3);
    }


    public void close() {
        servo_odometrie.setPosition(0);
    }

    public void open() {
        servo_odometrie.setPosition(0);
    }


}




