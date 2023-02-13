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

@Autonomous(name = "Autonom_StangaNOU_Ploiesti 2k23", group = "Robot")
public class TestAutonom extends LinearOpMode {

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
            x=20;
            y=14;

        } else if (parkingPosition == SleeveDetection.ParkingPosition.CENTER) {
            x=42;
            y=14;
        }

        else if (parkingPosition == SleeveDetection.ParkingPosition.LEFT){
            x=63;
            y=14;
        }

        Pose2d startPose = new Pose2d(36.00, 63.0, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(16, 63))
                .lineToLinearHeading(new Pose2d(17, 48.5, Math.toRadians(-119)))
                .UNSTABLE_addTemporalMarkerOffset(0,bratModule::goUp)
                .UNSTABLE_addTemporalMarkerOffset(1,glisieraModule::goUp)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(1,intake::open)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(18, 54, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0,glisieraModule::goDown)
                .UNSTABLE_addTemporalMarkerOffset(0,bratModule::autonom)
                //.lineToLinearHeading(new Pose2d(16, 35, Math.toRadians(0.0)))
                .lineToLinearHeading(new Pose2d(16, 14, Math.toRadians(180.0)))
                .lineToLinearHeading(new Pose2d(65.5, 13.8, Math.toRadians(180.0)))
                //.lineToLinearHeading(new Pose2d(x, y, Math.toRadians(180.)))
                .UNSTABLE_addTemporalMarkerOffset(1,intake::close)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1,bratModule::goUp)
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(52.3, 14.2, Math.toRadians(180.0)))
                .turn(Math.toRadians(32))
                .UNSTABLE_addTemporalMarkerOffset(1,glisieraModule::goUp)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(1,intake::open)
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(57.3, 14.2, Math.toRadians(212.0)))
                .UNSTABLE_addTemporalMarkerOffset(0,glisieraModule::goDown)
                .UNSTABLE_addTemporalMarkerOffset(0,bratModule::autonom)
                .turn(Math.toRadians(-32))
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(180.0)))
                .build();




        drive.followTrajectorySequenceAsync(traj1);


        while (opModeIsActive()) {
            drive.update();
            glisieraModule.update();
        }
    }
}