package org.firstinspires.ftc.teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO TEST")
public class Servo_test extends LinearOpMode {
    Servo servo = null;
    double modifier = .001;

    double change_modifier = 0.00005;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo_test");
        servo.setPosition(0.5);
        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.a){
                servo.setPosition(servo.getPosition() + modifier);
            }
            else if(gamepad1.b){
                servo.setPosition(servo.getPosition() - modifier);
            }
            if(gamepad1.dpad_up){
                modifier += change_modifier;
            }
            else if(gamepad1.dpad_down){
                modifier -= change_modifier;
            }

            telemetry.addData("POZITIE SERVO", servo.getPosition());
            telemetry.addData("MODIFIER", modifier);
            telemetry.update();

            // Start pos = 0.35
            // Inchis pos = 0.6

        }
    }


}


