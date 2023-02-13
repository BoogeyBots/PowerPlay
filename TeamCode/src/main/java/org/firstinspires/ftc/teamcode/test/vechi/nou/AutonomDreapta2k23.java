package org.firstinspires.ftc.teamcode.test.vechi.nou;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.BratSModule;
import org.firstinspires.ftc.teamcode.modules.GlisieraModule;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.IntakeModule;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.DetectionClass;
import org.firstinspires.ftc.teamcode.vision.pipelines.SleeveDetection;

@Autonomous(name = "Autonom_DreaptaNOU_Ploiesti 2k23", group = "Robot")
public class AutonomDreapta2k23 extends LinearOpMode {

    public static double DISTANCE = 4;
    Servo servo_gheare = null;
    public Servo servoST = null;
    public Servo servoDR = null;
    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;

    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;

    int x,y;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeModule intake = new IntakeModule(hardwareMap);
        GlisieraModule glisieraModule = new GlisieraModule(hardwareMap);
        BratSModule bratModule = new BratSModule(hardwareMap);
        DetectionClass detectionClass = new DetectionClass(hardwareMap);

        SleeveDetection.ParkingPosition parkingPosition = SleeveDetection.ParkingPosition.LEFT;

        servoDR = hardwareMap.get(Servo.class, "servo_brat_dr");
        servoST = hardwareMap.get(Servo.class, "servo_brat_st");
        servo_gheare = hardwareMap.get(Servo.class, "servo_gheara");


        detectionClass.init();
        intake.init();
        glisieraModule.init();
        bratModule.init();

        intake.close();

        while(!opModeIsActive()) {
            parkingPosition = detectionClass.getPosition();
        }

        waitForStart();

        if (parkingPosition == SleeveDetection.ParkingPosition.RIGHT) {
            x=-16;
            y=36;

        } else if (parkingPosition == SleeveDetection.ParkingPosition.CENTER) {
            x=-38;
            y=35;
        }

        else if (parkingPosition == SleeveDetection.ParkingPosition.LEFT){
            x=-62;
            y=35;
        }

        Pose2d startPose = new Pose2d(-36.00, 63.0, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-16, 63))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-17, 48.5, Math.toRadians(-61)))
                .UNSTABLE_addTemporalMarkerOffset(1,bratModule::goUp)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1,glisieraModule::goUp)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(1,intake::open)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-18, 54, Math.toRadians(-61)))
                .UNSTABLE_addTemporalMarkerOffset(1,glisieraModule::goDown)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(1,bratModule::goDown)
                .waitSeconds(2)
                .turn(Math.toRadians(-29))
                .lineTo(new Vector2d(-16, 35))
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .lineTo(new Vector2d(x,y))
                .build();




        drive.followTrajectorySequenceAsync(traj1);


        while (opModeIsActive()) {
            drive.update();
            glisieraModule.update();
        }
    }
}