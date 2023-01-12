package org.firstinspires.ftc.teamcode.test.vechi.nou;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClaw extends LinearOpMode {
    private Servo servo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo_gheara");
        }
}
