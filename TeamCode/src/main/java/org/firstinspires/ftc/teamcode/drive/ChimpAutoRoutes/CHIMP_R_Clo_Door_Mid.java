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

@Autonomous(group = "Chimps",name = "CHIMP_R_Clo_Door_Mid")
public class CHIMP_R_Clo_Door_Mid extends LinearOpMode {
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
        wBot.initForAuton("R_Clo_Door_Mid");
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
        int slideLength = 850;
        Pose2d firstPlacement = new Pose2d();
        Pose2d yellowPixelPlacement = new Pose2d();
        Pose2d firstWhitePixel = new Pose2d();

        Pose2d startPos = wBot.startingPosition;
        Pose2d startPos2 = wBot.startingPosition2;
        Pose2d laneAlignment = wBot.DoorAlignmentBoard;
        Pose2d doorStack = wBot.doorStack;
        Pose2d turn2Board = wBot.Turn2BoardMid;
        Pose2d parkPrepare = wBot.parkPrepare;
        Pose2d parkFinish = wBot.parkFinish;

        while (opModeInInit() && StopSearch == 0) {


            if ((OpenCVDetectTeamProp.centerX >240) && (OpenCVDetectTeamProp.centerY > 80) && (OpenCVDetectTeamProp.centerY < 160)) {
                zoneDetected = 3;
            } else if ((OpenCVDetectTeamProp.centerX < 60) && (OpenCVDetectTeamProp.centerY > 80) && (OpenCVDetectTeamProp.centerY < 160)) {
                zoneDetected = 1;
            } else if ((OpenCVDetectTeamProp.centerX > 130) && (OpenCVDetectTeamProp.centerY > 100) && (OpenCVDetectTeamProp.centerY < 140) && (OpenCVDetectTeamProp.centerX < 190)) {
                zoneDetected = 2;
            }

            if (!(zoneDetected==0)) {
                detections[0] = zoneDetected;
            }
            for(int x =19; x>0; x--) {
                detections[x] = detections[x-1];
            }
            if(opModeIsActive()){
                StopSearch = 1;

                for(int j = 19; j>0; j--){
                    if(detections[j] == 1){Zone1detections++;}
                    if(detections[j] == 2){Zone2detections++;}
                    if(detections[j] == 3){Zone3detections++;}
                }


                telemetry.addLine("Search Stopped" );
                updateTelemetry(telemetry);


            }
            if(Zone1detections > Zone2detections && Zone1detections > Zone3detections){zoneDetected=1;}
            else if(Zone2detections > Zone1detections && Zone2detections > Zone3detections){zoneDetected=2;}
            else if(Zone3detections > Zone1detections && Zone3detections > Zone2detections){zoneDetected=3;}
            /*
            if (zoneDetected == 1) {
                firstPlacement = wBot.firstPlacementLeft;
                yellowPixelPlacement = wBot.PerpendicularBoardLeft;
                firstWhitePixel = wBot.PerpendicularBoardRight;
            } else if (zoneDetected == 2) {
                firstPlacement = wBot.firstPlacementMid;
                yellowPixelPlacement = wBot.PerpendicularBoardMid;
                firstWhitePixel = wBot.PerpendicularBoardRight;
            } else //if (zoneDetected == 3)
            {
                firstPlacement = wBot.firstPlacementRight;
                yellowPixelPlacement = wBot.PerpendicularBoardRight;
                firstWhitePixel = wBot.PerpendicularBoardLeft;
            }

             */

        }
        if (zoneDetected == 1) {
            firstPlacement = wBot.firstPlacementLeft;
            yellowPixelPlacement = wBot.PerpendicularBoardLeft;
            firstWhitePixel = wBot.PerpendicularBoardRight;
        } else if (zoneDetected == 2) {
            firstPlacement = wBot.firstPlacementMid;
            yellowPixelPlacement = wBot.PerpendicularBoardMid;
            firstWhitePixel = wBot.PerpendicularBoardRight;
        } else if (zoneDetected == 3)
        {
            firstPlacement = wBot.firstPlacementRight;
            yellowPixelPlacement = wBot.PerpendicularBoardRight;
            firstWhitePixel = wBot.PerpendicularBoardLeft;
        }
        waitForStart();
        //PosesAndActions firstExtendation = new PosesAndActions(wBot.startExtendFirstPlacementAfter, "");
        int firstPlaceSlidesPos = 0;
        double correctorPosFirstPlace = 0;
        double firstBoardSlideAngle = 0.35;
        //Pose2d parkPrepare = wBot.
        //Change Poses
/*
        if (zoneDetected == 1) {
                firstPlacement = wBot.firstPlacementLeft;
                yellowPixelPlacement = wBot.PerpendicularBoardLeft;
                firstWhitePixel = wBot.PerpendicularBoardRight;
        } else if (zoneDetected == 2) {
                firstPlacement = wBot.firstPlacementMid;
                yellowPixelPlacement = wBot.PerpendicularBoardMid;
                firstWhitePixel = wBot.PerpendicularBoardRight;
        } else if (zoneDetected == 3)
        {
                firstPlacement = wBot.firstPlacementRight;
                yellowPixelPlacement = wBot.PerpendicularBoardRight;
                firstWhitePixel = wBot.PerpendicularBoardLeft;
            }

 */
            //else{
            // firstPlacement = wBot.firstPlacementLeft;
            //}
            //zoneDetected = wBot.TeamPropDetectionReading();

//            telemetry.addData("purplePixelPlacement", purplePixelPlacement);
//            telemetry.addData("firstPlacement", firstPlacement);
            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update();
            //}
        while(opModeIsActive()){
            timeForAuton.reset();
            wBot.clawVAngle.setPosition(0.3);
            wBot.slideRAngle.setPosition(0.3);
            wBot.slideLAngle.setPosition(0.3);
            posesToGoTo.add(new PosesAndActions(startPos, ""));
            posesToGoTo.add(new PosesAndActions(startPos2, ""));
            posesToGoTo.add(new PosesAndActions(firstPlacement, ""));
            follower.init(posesToGoTo, isTest, true);
            follower.goToPoints(true);
            if(zoneDetected == 3) {
                slideLength = 450;
            }
            wBot.clawExtensionManager(slideLength, 0);
            sleep(1000);
            wBot.openRightClaw();
            sleep(1000);//TEMP GAP FOR TESTING ONLY
            wBot.perpendicularBoardPlacement(firstBoardSlideAngle, 700);

            posesToGoTo.clear();
            //posesToGoTo.add(new PosesAndActions(turn2Board, ""));
            posesToGoTo.add(new PosesAndActions(yellowPixelPlacement, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            sleep(500);//AnotherTESTGAP

            wBot.openLeftClaw();
            sleep(1000);
            wBot.slideToTarget(0,-1);
            sleep(300);

/*
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.boardBack, ""));
            posesToGoTo.add(new PosesAndActions(laneAlignment, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            sleep(1000);
            */
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(wBot.boardBack, ""));
            posesToGoTo.add(new PosesAndActions(laneAlignment, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            sleep(2000);
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(doorStack, ""));//Monkey Operated Bot
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            sleep(1000);
            wBot.clawExtensionManager(1100,2);
            sleep(1000);
            wBot.closeLeftClaw();
            wBot.closeRightClaw();
            sleep(700);
            wBot.slideRAngle.setPosition(0.4);
            wBot.slideLAngle.setPosition(0.4);
            wBot.slideToTarget(0,-1);
            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(laneAlignment, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);

            posesToGoTo.clear();
            //posesToGoTo.add(new PosesAndActions(wBot.boardBack, ""));
            posesToGoTo.add(new PosesAndActions(firstWhitePixel, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.perpendicularBoardPlacement(0.5, 600);
            wBot.openLeftClaw();
            wBot.openRightClaw();


            posesToGoTo.clear();
            posesToGoTo.add(new PosesAndActions(parkPrepare, ""));
            posesToGoTo.add(new PosesAndActions(parkFinish, ""));
            follower.reinit(posesToGoTo);
            follower.goToPoints(true);
            wBot.resetCLaw4Park();
            sleep(20000);

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






