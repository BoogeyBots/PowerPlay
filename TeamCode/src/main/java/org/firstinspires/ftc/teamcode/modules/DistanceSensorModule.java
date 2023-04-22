package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorModule {
    HardwareMap hardwareMap;

    public DistanceSensorModule(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public DistanceSensor distanceSensor = null;

    public void init(){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
    }

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}
