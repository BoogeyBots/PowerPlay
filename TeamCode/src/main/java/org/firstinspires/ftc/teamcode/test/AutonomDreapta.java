package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.DetectionClass;
import org.firstinspires.ftc.teamcode.vision.pipelines.SleeveDetection;

@Autonomous(name="AutonomDreapta", group="Robot")
public class AutonomDreapta extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotorEx glisiere = null;
    private Servo cleste = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 301.9;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_GLISIERA = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double LUNGIME_GLISIERA = 9.84;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_GLISIERA = (COUNTS_PER_MOTOR_REV_GLISIERA * DRIVE_GEAR_REDUCTION) / LUNGIME_GLISIERA;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system variables.
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        glisiere = hardwareMap.get(DcMotorEx.class, "glisiere");
        cleste = hardwareMap.get(Servo.class, "cleste");
        DetectionClass detectionClass = new DetectionClass(hardwareMap);
        SleeveDetection.ParkingPosition parkingPosition = SleeveDetection.ParkingPosition.LEFT;


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightBack.getCurrentPosition());
        telemetry.update();



        Glisiere glisiere = new Glisiere(hardwareMap);
        glisiere.init();

        detectionClass.init();

        cleste.setPosition(0.62);

        while(!opModeIsActive()) {
            parkingPosition = detectionClass.getPosition();
        }

        waitForStart();
        if(parkingPosition == SleeveDetection.ParkingPosition.LEFT) {
            sleep(500);

            //glisiere.setPosition(-2350);
            encoderDrive(DRIVE_SPEED, -27.55, -27.55, 4.0);
            encoderDrive(TURN_SPEED, -10.5, 10.5 , 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout\
            //encoderDrive(TURN_SPEED, -1.2, 1.2, 4);

            encoderDrive(DRIVE_SPEED, 34.0, 34.0, 5);//de adaugat
            sleep(500);

            //glisiere.setPosition(-1500);
            sleep(400);
            //cleste.setPosition(0.35);

            //encoderDrive(TURN_SPEED, -2.5, 2.5, 4);
            //encoderDrive(DRIVE_SPEED, -1.5, -1.5, 5);

            //sleep(500);

            glisiere.setPosition(0);

            //encoderDrive(DRIVE_SPEED, 6.5, 6.5, 4.0);

        }
        else if (parkingPosition == SleeveDetection.ParkingPosition.CENTER) {
            sleep(500);

            //glisiere.setPosition(-2350);
            encoderDrive(TURN_SPEED, -10.5, 10.5 , 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout\
            //encoderDrive(TURN_SPEED, -1.2, 1.2, 4);

            encoderDrive(DRIVE_SPEED, 35.0, 35.0, 5);//de adaugat

            sleep(500);

            //glisiere.setPosition(-1500);
            //sleep(400);
            //cleste.setPosition(0.35);

            glisiere.setPosition(0);

            //encoderDrive(DRIVE_SPEED, -1.0, -1.0, 4);
            //encoderDrive(TURN_SPEED, -1.2, 1.2, 4);
            //encoderDrive(DRIVE_SPEED, 3.0, 3.0, 4);
        }
        else {
            sleep(500);

            //glisiere.setPosition(-2350);
            encoderDrive(DRIVE_SPEED, 26.55, 26.55, 4.0);
            encoderDrive(TURN_SPEED, -10.5, 10.5 , 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout\
            encoderDrive(DRIVE_SPEED, 34.0, 34.0, 5);

            sleep(500);

            //glisiere.setPosition(-1500);
            sleep(400);
            //cleste.setPosition(0.35);

            glisiere.setPosition(0);

            //encoderDrive(DRIVE_SPEED, -1.7, -1.7, 4);
        }

        //encoderDrive(DRIVE_SPEED, -2, -2, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        //glisiere.setPosition(-1000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running. */

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftFront.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            leftBack.setTargetPosition((int) (newLeftTarget + 2.244));
            rightBack.setTargetPosition((int) (newRightTarget + 2.244));

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy()) &&
                    (leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Count per Inch: ", COUNTS_PER_INCH);
            telemetry.update();


            sleep(250);   // optional pause after each move.
        }
    }
}
