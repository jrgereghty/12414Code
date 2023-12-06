package org.firstinspires.ftc.teamcode.drive;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;




public class CameraInit {
    private OpenCvWebcam webcam;
    private HardwareMap hardwareMap;


    private OpenCVPipeline p1; // REAL pipeline
    //private samplePipeline2 p2; // FAKE pipeline

    public CameraInit(HardwareMap hw) { // hardware map from the base class is a parameter
        p1 = new OpenCVPipeline(); // initialize your pipeline classes
        //p2 = new samplePipeline2();

        hardwareMap = hw;    //Configure the Camera in hardwareMap
        int cameraMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Get camera from hardware map, replace 'camera' with what is in your controlhub
        webcam =
                OpenCvCameraFactory.getInstance()
                        .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(p1); // Setting the intial pipeline

        webcam.setMillisecondsPermissionTimeout(10500);

        // Streaming Frames
        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                   public void onError(int errorCode) {
                        telemetry.addLine("THIS IS DEATH");
                    }
                });
    }

    // Switching Between Pipelines
    /*
    public void switchToSecondPipeline(){
        webcam.setPipeline(p2);
    }
*/
    public void switchToFirstPipeline(){
        webcam.setPipeline(p1);
    }

    // Get information from pipeline
    public String getPipeline1Output(){
        return p1.getLocation();
    }
    public Mat getBinaryCamera(){return p1.getCamera();}

    // call stop at the end of the opMode.
    public void stop() {
        webcam.stopStreaming();
    }
}
