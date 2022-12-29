

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive Blue By Time", group="Robot")
@Disabled
public class AutoBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor LeftFrontDrive = null;
    private DcMotor RightFrontDrive = null;
    private DcMotor LeftBackDrive = null;
    private DcMotor RightBackDrive = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LeftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFrontDrive = hardwareMap.get(DcMotor.class, "RightFront");
        LeftBackDrive = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBackDrive = hardwareMap.get(DcMotor.class, "RightBack");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LeftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


        LeftFrontDrive.setPower(-TURN_SPEED);
        RightFrontDrive.setPower(TURN_SPEED);
        LeftBackDrive.setPower(-TURN_SPEED);
        RightBackDrive.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 3:  Drive Backward for 1 Second
        LeftFrontDrive.setPower(FORWARD_SPEED);
        RightFrontDrive.setPower(FORWARD_SPEED);
        LeftBackDrive.setPower(FORWARD_SPEED);
        RightBackDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        LeftFrontDrive.setPower(TURN_SPEED);
        RightFrontDrive.setPower(-TURN_SPEED);
        LeftBackDrive.setPower(TURN_SPEED);
        RightBackDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
            LeftFrontDrive.setPower(FORWARD_SPEED);
            RightFrontDrive.setPower(FORWARD_SPEED);
            LeftBackDrive.setPower(FORWARD_SPEED);
            RightBackDrive.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.4)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        LeftFrontDrive.setPower(0);
        RightFrontDrive.setPower(0);
        LeftBackDrive.setPower(0);
        RightBackDrive.setPower(0);
                telemetry.addData("Path", "Complete");
                telemetry.update();
                sleep(1000);
            }
        }
