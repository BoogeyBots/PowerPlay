package org.firstinspires.ftc.teamcode.test.vechi.nou;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.GlisieraModule;

@TeleOp(name = "PE CICLU")
public class TestCiclu extends LinearOpMode {



    Brat brat = new Brat(hardwareMap);

    Servo servo_gheare = null;

    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;

    public Servo servoST = null;
    public Servo servoDR = null;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GlisieraModule glisieraModule = new GlisieraModule(hardwareMap);

        servo_gheare = hardwareMap.get(Servo.class, "servo_gheara");
        servo_gheare.setPosition(0.0);

        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");

        servoST.setPosition(0.140);
        servoDR.setPosition(1.0 - 0.140);

        glisieraModule.init();

        waitForStart();

        while (opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -(gamepad1.left_stick_x),
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad1.a) {

                servoST.setPosition(0.140);
                servoDR.setPosition(1.0 - 0.140);
            } else if (gamepad1.b) {
                servoST.setPosition(0.70);
                servoDR.setPosition(1.0 - 0.70);

            }

            if (gamepad1.x){
                servo_gheare.setPosition(0.0);
            }
            else if(gamepad1.y){
                servo_gheare.setPosition(0.45);
            }

            if(gamepad1.right_bumper){
                glisieraModule.goUp();
            }
            if (gamepad1.left_bumper){
                glisieraModule.goDown();
            }
            glisieraModule.update();
        }

    }

}
