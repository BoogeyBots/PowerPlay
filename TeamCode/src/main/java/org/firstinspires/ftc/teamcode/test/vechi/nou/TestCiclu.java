package org.firstinspires.ftc.teamcode.test.vechi.nou;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.GlisieraModule;
import org.firstinspires.ftc.teamcode.modules.BratSModule;
import org.firstinspires.ftc.teamcode.modules.IntakeModule;

@TeleOp(name = "PE CICLU")
public class TestCiclu extends LinearOpMode {

    //GamepadEx gamepadEx = new GamepadEx(gamepad1);
    ElapsedTime elapsedTime = new ElapsedTime();


    Brat brat = new Brat(hardwareMap);

    Servo servo_gheare = null;

    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;

    public Servo servoST = null;
    public Servo servoDR = null;

    boolean con_detectat = false;

    public DistanceSensor sensorRange = null;

    public double mod = 0.0;

    public double modServo = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GlisieraModule glisieraModule = new GlisieraModule(hardwareMap);
        BratSModule bratModule = new BratSModule(hardwareMap);
        IntakeModule intake = new IntakeModule(hardwareMap);

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        servo_gheare = hardwareMap.get(Servo.class, "servo_gheara");
        servo_gheare.setPosition(0.0);

        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");

        servoST.setPosition(0.125);//130
        servoDR.setPosition(1.0 - 0.125);

        glisieraModule.init();
        bratModule.init();
        intake.init();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_trigger > 0.1) {
                mod = 1.0;
            }
            else {
                mod = 0.5;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*mod,
                            -(gamepad1.left_stick_x)*mod,
                            -gamepad1.right_stick_x*mod
                    )
            );

            drive.update();

            if (gamepad2.a) {
                bratModule.goDown();
            } else if (gamepad2.b) {
                bratModule.goUp();
            }
            else if (gamepad2.dpad_up)
            {
                bratModule.goBlack();
            }

            if (gamepad2.x){
                intake.open();
            //    elapsedTime.reset();
              //  con_detectat = false;
            }
            else if(gamepad2.y){
                intake.close();
            }
    
            if(gamepad2.right_bumper){
                glisieraModule.goUp();
            }
            if (gamepad2.left_bumper) {
                glisieraModule.goDown();
            }
            if(gamepad2.dpad_down)
            {
                glisieraModule.goMid();
            }
            glisieraModule.update();

           /* if((sensorRange.getDistance(DistanceUnit.CM) < 3.0 && !con_detectat) && elapsedTime.milliseconds() > 500.0)
            {
                intake.close();
                con_detectat = true;
            }
            telemetry.addData("cm",sensorRange.getDistance(DistanceUnit.CM));
            telemetry.update();*/
        }

    }

}
