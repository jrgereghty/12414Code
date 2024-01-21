package org.firstinspires.ftc.teamcode.LevineLocal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVGreatestColorTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "Center Stage")
public class      BlueAuton extends LinearOpMode {
    OpenCvCamera webcam; //webcam2;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    PointFollower follower = new PointFollower(this);
    public static boolean isTest = false;
    public static boolean isParkFinal = true;
    @Override
    public void runOpMode() throws InterruptedException {



        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        PosesAndActions startingPostion = new PosesAndActions(new Pose2d(24, 35, Math.toRadians(90)), "");

        PosesAndActions beacon1Preload = new PosesAndActions(new Pose2d(24, 45, Math.toRadians(90)), "");

        PosesAndActions beacon2Preload = new PosesAndActions(new Pose2d(14, 37, Math.toRadians(90)), "");

        PosesAndActions beacon3Preload = new PosesAndActions(new Pose2d(3, 39, Math.toRadians(90)), "");

        PosesAndActions placementBeacon1 = new PosesAndActions(new Pose2d(12.5, 63, Math.toRadians(90)), "");

        PosesAndActions placementBeacon2 = new PosesAndActions(new Pose2d(12.5, 63, Math.toRadians(90)), "");

        PosesAndActions placementBeacon3 = new PosesAndActions(new Pose2d(12.5, 63, Math.toRadians(90)), "");

        PosesAndActions preloadPlacement = new PosesAndActions(new Pose2d(), "");

        Pose2d beacon3LineUpAfterTruss = new Pose2d(13, 39, Math.toRadians(90));

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        //webcam2.setPipeline(pipeline);
        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerBlue, OpenCVGreatestColorTest.upperBlue);
        webcam.setPipeline(colorPipe);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        int zoneDetected = 0;

        while (opModeInInit()) {
            if(!OpenCVDetectTeamProp.isDetected){
                zoneDetected = 1;
            }
            else if(OpenCVDetectTeamProp.centerX < 160){
                zoneDetected = 2;
            }
            else if(OpenCVDetectTeamProp.centerX > 160){
                zoneDetected = 1;
            }

            if (zoneDetected == 1) {
                preloadPlacement = beacon1Preload;
            } else if (zoneDetected == 2) {
                preloadPlacement = beacon2Preload;
            } else {
                preloadPlacement = beacon3Preload;
            }
            telemetry.addData("preloadPlacement", preloadPlacement);
            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
        }

        while (opModeIsActive()) {
            posesToGoTo.add(startingPostion);
            if (zoneDetected == 3) {
                posesToGoTo.add(new PosesAndActions(beacon3LineUpAfterTruss, ""));
            }
            posesToGoTo.add(preloadPlacement);

            follower.init(posesToGoTo, isTest);

            follower.goToPoints(true);
            sleep(700);
            terminateOpModeNow();
        }
    }
}