package org.firstinspires.ftc.teamcode.test.vechi;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO TEST")
public class Servo_test extends LinearOpMode {
    Servo servo = null;
    double modifier = 0.0001;

    double change_modifier = 0.00000005;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo_gheara");
        servo.setPosition(0.2);
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

            if(gamepad1.left_trigger>0.1) {
                servo.setPosition(servo.getPosition() - modifier * gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger>0.1) {
                servo.setPosition(servo.getPosition() + modifier  * gamepad1.right_trigger);
            }


            telemetry.addData("POZITIE SERVO", servo.getPosition());
            telemetry.addData("MODIFIER", modifier);
            telemetry.addData("CHANGE MODIFIER", change_modifier);

            telemetry.update();
        }
    }
}





