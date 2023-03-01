package org.firstinspires.ftc.teamcode.test.vechi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="TeleOP_Liliac", group="Robot")
public class TeleOPLiliac extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public Servo aripiStanga = null;
    public Servo aripiDreapta = null;
    public DcMotorEx lift = null;
    static final double COUNTS_PER_INCH = 301.9;

    public ElapsedTime timer = new ElapsedTime();



    @Override
    public void runOpMode() {
        double left;
        double right;
        double y;
        double x;
        double max;
        double rx;
        double modifier = 0.1;
        int liftModifier = 100;

        // Define and Initialize Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFront");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "RightFront");
        aripiDreapta = hardwareMap.get(Servo.class, "AripiDreapta");
        aripiStanga = hardwareMap.get(Servo.class, "AripiStanga");
        lift = hardwareMap.get(DcMotorEx.class, "Lift");

        // To y forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test y.
        // Note: The settings here assume direct y on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        lift.setTargetPosition(0);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            y = -gamepad1.left_stick_y*.6;
            x  =  gamepad1.right_stick_x*.5;


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

            leftFrontDrive.setPower(left);
            rightFrontDrive.setPower(right);
            // Use gamepad left & right Bumpers to open and close the claw

            // Move both servos to new position.  Assume servos are mirror image of each other.

            lift.setPower(0.9);

            if(gamepad1.dpad_up) {
                modifier+=0.1;
            }
            else if(gamepad1.dpad_down) {
                modifier-=0.1;
            }

            if (gamepad1.dpad_left ) {
                lift.setTargetPosition(lift.getCurrentPosition() + liftModifier);
            }
            else if (gamepad1.dpad_right) {
                lift.setTargetPosition(lift.getCurrentPosition() - liftModifier);
            }
            /*
            else {
                lift.setTargetPosition(lift.getCurrentPosition());
            }

             */
            if(gamepad1.dpad_up && timer.milliseconds() > 500.0) {
                liftModifier += 100;
                timer.reset();
            }

            if(gamepad1.y) {
                aripiDreapta.setPosition(0.615);
                aripiStanga.setPosition(0.75);
            }
            if(gamepad1.a) {
                aripiDreapta.setPosition(0.215);
                aripiStanga.setPosition(1.0);
            }

            telemetry.addData("CurrentPositionSTANGA: ", aripiStanga.getPosition());
            telemetry.addData("CurrentPositionDREAPTA: ", aripiDreapta.getPosition());
            telemetry.addData("Modifier: ", liftModifier);
            telemetry.addData("Lift: ", lift.getCurrentPosition());
            telemetry.update();



            // Use gamepad left & right Bumpers to open and close the claw

            //telemetry.addData("right", "%.2f", right);
            //telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
