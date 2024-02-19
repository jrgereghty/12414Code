package org.firstinspires.ftc.teamcode.drive.ChimpAutoRoutes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVGreatestColorTest;
import org.firstinspires.ftc.teamcode.LevineLocalization.ActionRunnerCenterStageAuton;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.drive.JayMap;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "Chimps", name = "CHIMP_B_Clo_Door_Mid_L")
public class CHIMP_B_Clo_Door_Mid_L extends LinearOpMode {
    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    JayMap wBot = new JayMap(this);

    public static int[] detections = new int[20];
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, wBot);
    PointFollower follower = new PointFollower(this, actionRunner);
    public static boolean isTest = false;
    public static boolean isParkFinal = true;
    public ElapsedTime timeForAuton = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        wBot.init();
        wBot.initForAuton("B_Clo_Door_Mid_L");
        //wBot.resetSlides();
        ArrayList<PosesAndActions> posesToGoTo = new ArrayList<>();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        colorPipe = new OpenCVDetectTeamProp(telemetry, OpenCVGreatestColorTest.lowerRed, OpenCVGreatestColorTest.upperRed);
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

        int StopSearch = 0;
        int Zone1detections = 0;
        int Zone2detections = 0;
        int Zone3detections = 0;

        int zoneDetected = 0;
        while (opModeInInit() && StopSearch == 0) {


            if ((OpenCVDetectTeamProp.centerX > 240) && (OpenCVDetectTeamProp.centerY > 80) && (OpenCVDetectTeamProp.centerY < 160)) {
                zoneDetected = 3;

            } else if ((OpenCVDetectTeamProp.centerX < 60) && (OpenCVDetectTeamProp.centerY > 80) && (OpenCVDetectTeamProp.centerY < 160)) {
                zoneDetected = 1;

            } else if ((OpenCVDetectTeamProp.centerX > 130) && (OpenCVDetectTeamProp.centerY > 100) && (OpenCVDetectTeamProp.centerY < 140) && (OpenCVDetectTeamProp.centerX < 190)) {
                zoneDetected = 2;

            }


            if (!(zoneDetected == 0)) {
                detections[0] = zoneDetected;
            }
            for (int x = 19; x > 0; x--) {
                detections[x] = detections[x - 1];
            }
            if (opModeIsActive()) {
                StopSearch = 1;

                for (int j = 19; j > 0; j--) {
                    if (detections[j] == 1) {
                        Zone1detections++;
                    }
                    if (detections[j] == 2) {
                        Zone2detections++;
                    }
                    if (detections[j] == 3) {
                        Zone3detections++;
                    }

                }


                telemetry.addLine("Search Stopped");
                updateTelemetry(telemetry);


            }
            if (Zone1detections > Zone2detections && Zone1detections > Zone3detections) {
                zoneDetected = 1;
            } else if (Zone2detections > Zone1detections && Zone2detections > Zone3detections) {
                zoneDetected = 2;
            } else if (Zone3detections > Zone1detections && Zone3detections > Zone2detections) {
                zoneDetected = 3;
            }
        }

        //waitForStart();

        //PosesAndActions firstExtendation = new PosesAndActions(wBot.startExtendFirstPlacementAfter, "");
        int firstPlaceSlidesPos = 0;
        double correctorPosFirstPlace = 0;
        double firstBoardSlideAngle = 0.35;
        Pose2d firstPlacement = new Pose2d();
        Pose2d yellowPixelPlacement = new Pose2d();
        Pose2d firstWhitePixel = new Pose2d();

        Pose2d startPos = wBot.startingPosition;
        Pose2d laneAlignment = wBot.DoorAlignmentBoard;
        Pose2d doorStack = wBot.doorStack;
        //Change Poses
        if(zoneDetected == 1){
            firstPlacement = wBot.firstPlacementLeft;
            yellowPixelPlacement = wBot.PerpendicularBoardLeft;
            firstWhitePixel = wBot.PerpendicularBoardRight;
        }
        else if(zoneDetected == 2){
            firstPlacement = wBot.firstPlacementMid;
            yellowPixelPlacement = wBot.PerpendicularBoardMid;
            firstWhitePixel = wBot.PerpendicularBoardRight;
        }
        else if(zoneDetected == 3){
            firstPlacement = wBot.firstPlacementRight;
            yellowPixelPlacement = wBot.PerpendicularBoardRight;
            firstWhitePixel = wBot.PerpendicularBoardLeft;
        }
        else{
            firstPlacement = wBot.firstPlacementLeft;
        }
        while(opModeIsActive()){
            //zoneDetected = wBot.TeamPropDetectionReading();


            if(zoneDetected == 3){
                /*
                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
                firstPlacement = wBot.firstPlacementBeacon1After;
                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon1";
                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePosBeacons13;
                correctorPosFirstPlace = MonkeyMap.correctorServoBeacon1PreloadPlace;

                 */
            }
            else if(zoneDetected == 2){
                /*
                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon23;
                firstPlacement = wBot.firstPlacementBeacon2After;
                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon2";
                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePosBeacon2;
                correctorPosFirstPlace = MonkeyMap.correctorServoMidPos;

                 */
            }
            else{
                /*
                purplePixelPlacement = wBot.purplePixelPlacementAfterFarAndCloseBeacon1;
                firstPlacement = wBot.firstPlacementBeacon3After;
                firstExtendation.action = "extendSlidesFirstPlacementAfterBeacon1";
                firstPlaceSlidesPos = MonkeyMap.slidesFirstPlacePosBeacons13;
                correctorPosFirstPlace = MonkeyMap.correctorServoBeacon3PreloadPlace;

                 */
            }
//            telemetry.addData("purplePixelPlacement", purplePixelPlacement);
//            telemetry.addData("firstPlacement", firstPlacement);
            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
        }

        while(opModeIsActive()){

            timeForAuton.reset();
            //wBot.closeGrabber();
            //wBot.setFlipperPos(MonkeyMap.flipperPosUpPurplePixels);
            //wBot.setRotatorFlush();



            if(zoneDetected == 3) {

                //wBot.extendSlidesFarBeaconAfter();
            }
            if(zoneDetected == 2){
                //wBot.extendSlidesMidBeaconAfter();
                //wBot.correctorServo.setPosition(MonkeyMap.correctorServoBeacon2BeforePos);
            }
            if(zoneDetected == 1){
                //wBot.extendSlidesCloseBeaconAfter();
            }
            posesToGoTo.add(new PosesAndActions(startPos, ""));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.init(posesToGoTo, isTest);
            follower.goToPoints(true);
            wBot.clawExtensionManager(500,0);
            wBot.openRightClaw();
            sleep(3000);//TEMP GAP FOR TESTING ONLY
            posesToGoTo.clear();
            if(zoneDetected == 1){
            posesToGoTo.add(new PosesAndActions(wBot.reset4Left,""));}
            posesToGoTo.add(new PosesAndActions(yellowPixelPlacement,""));
            follower.init(posesToGoTo, isTest);
            follower.goToPoints(true);
            wBot.perpendicularBoardPlacement(firstBoardSlideAngle);
            wBot.openLeftClaw();
            sleep(3000);//AnotherTESTGAP
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(laneAlignment,""));
            posesToGoTo.add(new PosesAndActions(doorStack,""));
            follower.init(posesToGoTo, isTest);
            follower.goToPoints(true);
            sleep(2000);
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(laneAlignment,""));
            posesToGoTo.add(new PosesAndActions(firstWhitePixel,""));
            follower.init(posesToGoTo, isTest);
            follower.goToPoints(true);

            /*
            sleep(MonkeyMap.sleepTimeExtendSlides);
            wBot.openRightGrabber();
            sleep(MonkeyMap.sleepTimePlacePurplePixel);
            wBot.setCorrectorMid();
            wBot.resetSlides();
            wBot.flipUpFirstPlace();

             */
//            wBot.rotatorServo.setPosition(MonkeyMap.rotatorServoFirstPlace);
//            wBot.correctorServo.setPosition(correctorPosFirstPlace)
            /*
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.startExtendFirstPlacementAfter, ""));
            posesToGoTo.add(new PosesAndActions(wBot.turnForFirstPlacementAfter, ""));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.setAutoRotator(wBot.flipperServoLeft.getPosition());
            sleep(MonkeyMap.sleepTimeWaitForFlipFirstPlace);
            wBot.encodedSlipperySlides(firstPlaceSlidesPos, MonkeyMap.slidePowerEncoder);
            sleep(MonkeyMap.sleepTimeExtendSlides);
            wBot.openLeftGrabber();
            sleep(MonkeyMap.sleepTimeYellowPixel);
            wBot.resetArm();

             */
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 0, true, false);
//            wBot.placeInAutonFar(follower, posesToGoTo, false);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, true, false);
//            wBot.placeInAutonFar(follower, posesToGoTo, false);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 0, true, true);
//            wBot.placeInAutonFar(follower, posesToGoTo);
//            wBot.pickUpInAutonFar(follower, posesToGoTo, 1, true, true);
//            wBot.placeInAutonFar(follower, posesToGoTo);

//            sleep(sleepTimeTestAuton);

//            for (int i = 0; i < 2; i++) {
//                wBot.autonLoopFar(follower, posesToGoTo, wBot.wrapPixelTypeInt(i), true, i>1);
//            }
//            telemetry.addData("Time for auton ", timeForAuton);
//            telemetry.update();
            terminateOpModeNow();
        }
    }
}





