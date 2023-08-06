package org.firstinspires.ftc.teamcode.drive.offroad_robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.test.vechi.Glisiere;


@TeleOp(name="Birsan e offroad")
public class TeleOP_offroad extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;


    @Override
    public void runOpMode() {
        double left;
        double right;
        double y;
        double x;
        double max;
        double rx;
        // Define and Initialize Motors
        leftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");
        rightMotor  = hardwareMap.get(DcMotor.class, "RightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // To y forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test y.
        // Note: The settings here assume direct y on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just y straight, or just x.

            y = -gamepad1.left_stick_y*.9;
            x  =  gamepad1.right_stick_x*.9;

            while(gamepad1.left_stick_y>0.1){
                leftMotor.setPower(gamepad1.left_stick_y);
            }
            while(gamepad1.right_stick_y>0.1){
                rightMotor.setPower(gamepad1.right_stick_y);
            }

            /*
            // Combine y and x for blended motion.

            left  = y + x;
            right = y - x;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(y), Math.abs(x));
            if (max > 1.0)
            {
                left = (y+x) / max;
                right = (y-x) / max;
            }

            // Output the safe vales to the motor drives.

            leftMotor.setPower(left);
            rightMotor.setPower(left);

            // Use gamepad left & right Bumpers to open and close the claw


            telemetry.addData("CurrentPosition: ", leftMotor.getCurrentPosition());
            telemetry.addData("CurrentPosition: ", rightMotor.getCurrentPosition());
            telemetry.update();

            // Use gamepad left & right Bumpers to open and close the claw

            //telemetry.addData("right", "%.2f", right);
            //telemetry.update();

            // Pace this loop so jaw action is reasonable speed.

             */
            sleep(50);
        }
    }
}
