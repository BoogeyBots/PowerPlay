package org.firstinspires.ftc.teamcode.test.vechi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.BratSModule;
import org.firstinspires.ftc.teamcode.modules.DistanceSensorModule;
import org.firstinspires.ftc.teamcode.modules.GlisieraModule;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.IntakeModule;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.DetectionClass;
import org.firstinspires.ftc.teamcode.vision.pipelines.SleeveDetection;

@Autonomous(name = "STANGA NICUSORAUTONOM !BUBUIE!", group = "Robot")
public class NICUSORAUTONOMNUINCERCATIBUBUIE extends LinearOpMode {

    public static double DISTANCE = 4;
    Servo servo_gheare = null;
    public Servo servoST = null;
    public Servo servoDR = null;
    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;

    public boolean usingSensor = false;

    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;

    int x, y, nr=0;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeModule intake = new IntakeModule(hardwareMap);
        GlisieraModule glisieraModule = new GlisieraModule(hardwareMap);
        BratSModule bratModule = new BratSModule(hardwareMap);
        DetectionClass detectionClass = new DetectionClass(hardwareMap);
        DistanceSensorModule distanceSensorModule = new DistanceSensorModule(hardwareMap);

        SleeveDetection.ParkingPosition parkingPosition = SleeveDetection.ParkingPosition.LEFT;

        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");
        servo_gheare = hardwareMap.get(Servo.class, "servo_gheara");

        detectionClass.init();
        intake.initializare();
        glisieraModule.init();
        bratModule.init();
        distanceSensorModule.init();

        intake.close();

        detectionClass.startFlashlight();

        while (!opModeIsActive()) {
            parkingPosition = detectionClass.getPosition();

        }

        detectionClass.stopFlashlight();

        waitForStart();

        if (parkingPosition == SleeveDetection.ParkingPosition.RIGHT) {
            x = 17;
            y = 14;

        } else if (parkingPosition == SleeveDetection.ParkingPosition.CENTER) {
            x = 41;
            y = 14;
        } else if (parkingPosition == SleeveDetection.ParkingPosition.LEFT) {
            x = 63;
            y = 14;
        }

        Pose2d startPose = new Pose2d(36., 63.0, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(38, 63))
                .UNSTABLE_addTemporalMarkerOffset(0.5, bratModule::goUp)
                .lineTo((new Vector2d(38, 25.5)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, glisieraModule::goUp)
                .turn(Math.toRadians(-27.5))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, intake::open)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, bratModule::autonom)
                .UNSTABLE_addTemporalMarkerOffset(1, glisieraModule::goDown)
                .lineToLinearHeading(new Pose2d(38, 12, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->setUsingSensor(true))
                .lineToLinearHeading(new Pose2d(64.8, 13., Math.toRadians(180.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->setUsingSensor(false))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, intake::close)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, bratModule::goUp)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(51.8, 14.23, Math.toRadians(-180.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.6, glisieraModule::goUp)
                .turn(Math.toRadians(32))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, intake::open)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(57.3, 14.2, Math.toRadians(212.0)))
                .UNSTABLE_addTemporalMarkerOffset(0, glisieraModule::goDown)
                .UNSTABLE_addTemporalMarkerOffset(0, bratModule::autonom2)
                .turn(Math.toRadians(-32))
                .UNSTABLE_addTemporalMarkerOffset(1.0, ()->setUsingSensor(true))
                .lineToLinearHeading(new Pose2d(64.9, 12.1, Math.toRadians(180.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.0, ()->setUsingSensor(false))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, intake::close)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, bratModule::goUp)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(51.8, 14.23, Math.toRadians(-180.0)))
                .UNSTABLE_addTemporalMarkerOffset(0.6, glisieraModule::goUp)
                .turn(Math.toRadians(33))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, intake::open)
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(57.3, 14.2, Math.toRadians(213.0)))
                .UNSTABLE_addTemporalMarkerOffset(0, glisieraModule::goDown)
                .UNSTABLE_addTemporalMarkerOffset(0, bratModule::autonom2)
                .turn(Math.toRadians(-33))
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(180.0)))


                .build();


        drive.followTrajectorySequenceAsync(traj1);


        while (opModeIsActive()) {
            drive.update();
            glisieraModule.update();
            if(distanceSensorModule.getDistance()<6.0 && usingSensor==true){
                intake.close();
                telemetry.addData("detectat:","da");
            }
            else
                telemetry.addData("detectat:","nu");
            telemetry.update();
        }
    }
    private void setUsingSensor(boolean either){
        usingSensor = either;
    }



}