package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance1 {
    HardwareMap hardwareMap;

    public Distance1(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public DistanceSensor sensorRange = null;

    public void init() {

        sensorRange = hardwareMap.get(DistanceSensor.class, "Sensor");

    }

    public void update(){

    }

    public void detect()
    {
        if(sensorRange.getDistance(DistanceUnit.CM) < 1.0){

        }
    }

}




