package org.firstinspires.ftc.teamcode.test.vechi.nou;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Brat {
    HardwareMap hardwareMap;
    public Brat(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public Servo servoST = null;
    public Servo servoDR = null;

    double resolution = 0.00005;
    double resChangeSpeed = 0.00000001;

    public void init () {
        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");

        servoST.setPosition(0.5);
        servoDR.setPosition(0.5);
    }

    public void goDown () {
        servoST.setPosition(0.1238);
        servoDR.setPosition(1.0 - 0.1238);
    }

    public void goUp () {
        servoST.setPosition(0.7344444444);
        servoDR.setPosition(1.0 - 0.7344444444);
    }


}
