package org.firstinspires.ftc.teamcode.Regionala;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "Dreapta_Regionala_Terminal", group = "Robot")
public class AutonomDreaptaRegionala extends LinearOpMode {

    public static double DISTANCE = 4;
    Servo servo_gheare = null;
    public Servo servoST = null;
    public Servo servoDR = null;
    public DcMotorEx motorDR_ENC = null;
    public DcMotorEx motorST = null;

    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;

    double x,y;

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
            x=-61.99;
            y=38.5;

        } else if (parkingPosition == SleeveDetection.ParkingPosition.CENTER) {
            x=-36;
            y=38.5;
        }

        else if (parkingPosition == SleeveDetection.ParkingPosition.LEFT){
            x=-12;
            y=38.5;
        }

        Pose2d startPose = new Pose2d(-36.00, 63.0, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36,62))
                .UNSTABLE_addTemporalMarkerOffset(0,bratModule::goBlack)
                .lineTo(new Vector2d(-62, 62))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,intake::open)
                .lineTo(new Vector2d(-62,38.5))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(x, y, Math.toRadians(0.0)))
                .build();




        drive.followTrajectorySequenceAsync(traj1);


        while (opModeIsActive()) {
            drive.update();
            glisieraModule.update();
        }
    }
}