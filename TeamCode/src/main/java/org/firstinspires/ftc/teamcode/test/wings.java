package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="aripi", group="Robot")
public class wings extends LinearOpMode {

    private Servo aripiStanga = null;
    private Servo aripiDreapta = null;

    @Override
    public void runOpMode() throws InterruptedException {
        aripiStanga = hardwareMap.get(Servo.class, "AripiStanga");
        aripiDreapta = hardwareMap.get(Servo.class, "AripiDreapta");



        waitForStart();

        aripiStanga.setPosition(0.1);
        aripiDreapta.setPosition(-0.1);
    }

}
