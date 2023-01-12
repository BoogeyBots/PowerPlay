package org.firstinspires.ftc.teamcode.test.vechi.nou;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DouaMotoare extends LinearOpMode {
    private DcMotorEx motor_st = null;
    private DcMotorEx motor_dr = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motor_st = hardwareMap.get(DcMotorEx.class, "motor_lift_st");
        motor_dr = hardwareMap.get(DcMotorEx.class, "motor_lift_dr");

        motor_st.setDirection(DcMotorEx.Direction.REVERSE);

        motor_dr.setVelocityPIDFCoefficients(15.0, 3.0, 0.0, 0.0);

        while (opModeIsActive()){
            motor_st.setVelocity(motor_dr.getVelocity());

        }
       // motor_dr.setTargetPosition();

    }
}
