package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ImpingatoareModule {
    HardwareMap hardwareMap;

    public ImpingatoareModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    Servo servo_imp = null;


    public void init() {
        servo_imp = hardwareMap.get(Servo.class, "imp");
        servo_imp.setPosition(0.8);
    }


    public void close() {
        servo_imp.setPosition(0.8);
    }

    public void open() {
        servo_imp.setPosition(0.2);
    }


}




