package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.pipelines.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class DetectionClass extends SleeveDetection{
    HardwareMap hardwareMap;
    SleeveDetection sleeveDetection = new SleeveDetection();
    OpenCvInternalCamera camera;
    String webcamName = "Webcam 1";

    public DetectionClass(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {}


        });
    }

    public void stopStreaming(){
        camera.stopStreaming();
    }

    public ParkingPosition getPosition(){
        return sleeveDetection.getPosition();
    }

    public void startFlashlight() {

        camera.setFlashlightEnabled(true);

    }

    public void stopFlashlight() {

        camera.setFlashlightEnabled(false);

    }


}
