package org.firstinspires.ftc.teamcode.drive.autonRoutesOld;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVGreatestColorTest;
import org.firstinspires.ftc.teamcode.drive.JayMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "71R_Far_Door_Edge_REVERSECOLOR")

public class R_Far_Door_Edge_Experimental extends LinearOpMode {


    OpenCvCamera webcam;
    JayMap wBot = new JayMap(this);
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    //PointFollower follower = new PointFollower(this);
    //public static boolean isTest = false;
    public static boolean isParkFinal = true;

    //public static double pos = 0.5;
    public static double hPos = 0.5;
    public static double vPos = 0.7;
    public static double lPos = 0;
    public static double rPos = 0;

    double slideLength = 0.0;
    double slidePos = 0.0;

    double slidePower;
    public static int[] detections = new int[20];




    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        wBot.init();
        wBot.initSlideToPos();




        Pose2d stackReady = new Pose2d(-45, -12, Math.toRadians(180));
        //Pose2d stackReady2 = new Pose2d(-45, 9.5, Math.toRadians(180));
        Pose2d laneAlignment = new Pose2d(36, -11, Math.toRadians(180));
        Pose2d startPose = new Pose2d(-34.50, -63.5, Math.toRadians(90));
        Pose2d parkPrepare = new Pose2d(45, -10, Math.toRadians(0));
        Pose2d parkFinish = new Pose2d(53, -10, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory start2leftspike = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-48, -30, Math.toRadians(0)))
                .addDisplacementMarker(4, ()->{
                    wBot.clawExtensionManager(800,1);
        })
                .build();
        Trajectory spike2whiteleft = drive.trajectoryBuilder(start2leftspike.end())
                .lineToSplineHeading(stackReady)//new Pose2d(-45, 10, Math.toRadians(180))
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(0,0.5,-1);
                })
                .addDisplacementMarker(10, ()->{
                    wBot.setSlideAngle(0.08);//0.103
                })
                .build();
        Trajectory back79 = drive.trajectoryBuilder(stackReady)
                .lineTo(new Vector2d(36,-13))
                .addDisplacementMarker(1, ()->{
                    wBot.slideToTarget(1000,0.3);
                })
                .addDisplacementMarker(1, ()->{
                    wBot.slideToTarget(0,-1);
                })
                .build();
        Trajectory white2boardpart2left = drive.trajectoryBuilder(back79.end())
                .lineToSplineHeading(new Pose2d(50, -16, Math.toRadians(-69)))//54.7
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(0,0.8,-1);
                })
                .addDisplacementMarker(15, ()->{
                    wBot.setAllSlidePoses(1420,0.27,1);
                })
                .build();
        Trajectory white2boardpart2leftPerpendicular = drive.trajectoryBuilder(back79.end())
                .lineToSplineHeading(new Pose2d(44.5, -33, Math.toRadians(0)))//54.7
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(0,0.8,-1);
                })
                .addDisplacementMarker(15, ()->{
                    wBot.setAllSlidePoses(300,0.27,1);
                    wBot.clawHAngle.setPosition(hPos);
                })
                .addDisplacementMarker(20, ()->{
                    wBot.slideToTarget(670,1);
                })
                .build();
        Trajectory white2boardpart2midPerpendicular = drive.trajectoryBuilder(back79.end())
                .lineToSplineHeading(new Pose2d(44.5, -37, Math.toRadians(0)))//54.7
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(0,0.8,-1);
                })
                .addDisplacementMarker(15, ()->{
                    wBot.setAllSlidePoses(300,0.27,1);
                    wBot.clawHAngle.setPosition(hPos);
                })
                .addDisplacementMarker(20, ()->{
                    wBot.slideToTarget(670,1);
                })
                .build();
        Trajectory white2boardpart2rightPerpendicular = drive.trajectoryBuilder(back79.end())
                .lineToSplineHeading(new Pose2d(44.5, -32, Math.toRadians(0)))//54.7
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(0,0.8,-1);
                })
                .addDisplacementMarker(15, ()->{
                    wBot.setAllSlidePoses(300,0.27,1);
                    wBot.clawHAngle.setPosition(hPos);
                })
                .addDisplacementMarker(20, ()->{
                    wBot.slideToTarget(670,1);
                })
                .build();
//Second Spike Mark - - - - - - - -  - - - - - - - -  - - - - -
        Trajectory start2midspike = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-56, -26, Math.toRadians(0.00)))
                .addDisplacementMarker(1, ()->{
                    wBot.clawExtensionManager(500,0);
                })
                .build();
        Trajectory spike2whitemid = drive.trajectoryBuilder(start2midspike.end())
                .lineToSplineHeading(stackReady)//new Pose2d(-45, 10, Math.toRadians(180))
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(0,0.5,-1);
                })
                .addDisplacementMarker(10, ()->{
                    wBot.setSlideAngle(0.08);//0.103
                })
                .build();

        Trajectory start2rightspike = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-48, -10, Math.toRadians(270)))//change angle @####@##@#@#@#@-34,54
                .addDisplacementMarker(1, ()->{
                    wBot.clawExtensionManager(300,0);
                })
                .build();

        Trajectory leftplace = drive.trajectoryBuilder(back79.end())
                .splineToConstantHeading(new Vector2d(48,-22), Math.toRadians(45)) //left 6
                .build();


        Trajectory spike2whiteright = drive.trajectoryBuilder(start2rightspike.end())
                .lineToSplineHeading(stackReady)//new Pose2d(-45, 10, Math.toRadians(180))
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(0,0.5,-1);
                })
                .addDisplacementMarker(10, ()->{
                    wBot.setSlideAngle(0.08);//0.103
                })
                .build();
        Trajectory white2boardpart2Mid = drive.trajectoryBuilder(back79.end())
                .lineToSplineHeading(new Pose2d(50, -16, Math.toRadians(-45)))
                //.addDisplacementMarker(1, ()->{
                 //   wBot.setAllSlidePoses(300,0.7,-1);
               // })
                .addDisplacementMarker(4, ()->{
                    wBot.setAllSlidePoses(1100,0.6,1);
                })
                .build();
        Trajectory white2boardpart2Right = drive.trajectoryBuilder(back79.end())
                .lineToSplineHeading(new Pose2d(50, -16, Math.toRadians(-38)))
                .addDisplacementMarker(1, ()->{
                    wBot.setAllSlidePoses(300,0.7,-1);
                })
                .addDisplacementMarker(10, ()->{
                    wBot.setAllSlidePoses(700,0.6,1);
                })
                .build();


        Trajectory leftParkPrepare = drive.trajectoryBuilder(white2boardpart2Mid.end())
                .lineToSplineHeading(parkPrepare)
                .addDisplacementMarker(3, ()->{
                    wBot.slideToTarget(0,-1);
                    wBot.resetCLaw4Park();
                })
                .build();
        Trajectory MidParkPrepare = drive.trajectoryBuilder(white2boardpart2Right.end())
                .lineToSplineHeading(parkPrepare)
                .addDisplacementMarker(5, ()->{
                    wBot.slideToTarget(0,-1);
                    wBot.resetCLaw4Park();
                })
                .build();
        Trajectory RightParkPrepare = drive.trajectoryBuilder(white2boardpart2Mid.end())
                .lineToSplineHeading(parkPrepare)
                .addDisplacementMarker(5, ()->{
                    wBot.slideToTarget(0,-1);
                    wBot.resetCLaw4Park();
                })
                .build();

        Trajectory parkEnding = drive.trajectoryBuilder(parkPrepare)
                .lineToSplineHeading(parkFinish)
                .build();



        //Spline Trajectories



        Trajectory return2midSetLeft = drive.trajectoryBuilder(white2boardpart2left.end())
                .lineToSplineHeading(new Pose2d(48, -16, Math.toRadians(70)))
                .build();
        Trajectory return2midLeft = drive.trajectoryBuilder(return2midSetLeft.end())
                .lineToSplineHeading(new Pose2d(36, -9, Math.toRadians(180)))
                .build();
        Trajectory return2midLeftP = drive.trajectoryBuilder(white2boardpart2leftPerpendicular.end())
                .lineToSplineHeading(new Pose2d(36, -9, Math.toRadians(180)))
                .build();
        Trajectory return2midMid = drive.trajectoryBuilder(white2boardpart2midPerpendicular.end())
                .lineToSplineHeading(new Pose2d(36, -9, Math.toRadians(180)))
                .build();

        Trajectory return2midRight = drive.trajectoryBuilder(white2boardpart2rightPerpendicular.end())
                .lineToSplineHeading(new Pose2d(36, -9, Math.toRadians(180)))
                .build();

        Trajectory return2whites = drive.trajectoryBuilder(laneAlignment)
                .lineToSplineHeading(stackReady)//new Pose2d(-45, 10, Math.toRadians(180))//new Vector2d(-45,10
                .build();

        //Trajectory return2boardfinal = drive.trajectoryBuilder(return2whites.end())
               // .lineTo(new Vector2d(36,9))
                //.build();

        /*
        Trajectory return2sender = drive.trajectoryBuilder(back11.end())
                .forward(95)
                .build();
        Trajectory return2sender2 = drive.trajectoryBuilder(return2sender.end())
                .lineToSplineHeading(startPose)
                .build();

         */











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




        //___________________________________________________________________








        int StopSearch = 0;
        int Zone1detections = 0;
        int Zone2detections = 0;
        int Zone3detections = 0;

        int zoneDetected = 0;
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


        }
        waitForStart();

        if (zoneDetected == 3) {
            wBot.clawVAngle.setPosition(0.35);
            wBot.setSlideAngle(0.2);
            /*
            wBot.slideLAngle.setPosition(0.10);
            wBot.slideRAngle.setPosition(0.10);

             */
            drive.followTrajectory(start2leftspike);
            //sleep(100);

            wBot.openRightClaw();//place
             // to white pixels, turned towards spike
            //sleep(100);
            drive.followTrajectory(spike2whiteleft);
            sleep(100);
            //clawVAngle.setPosition(0.28);
            wBot.clawVAngle.setPosition(0.34);
            //sleep(400);
            wBot.slideToTarget(750,1);
            wBot.setSlideAngle(0.103);
            //wBot.setSlideAngle(0.08);
            sleep(500);
            wBot.closeRightClaw(); // +1, hopefully
            sleep(300);


            drive.followTrajectory(back79);

            wBot.clawVAngle.setPosition(0.13);
            //wBot.clawHAngle.setPosition(0.25);
            //drive.followTrajectory(white2boardpart2left);
            drive.followTrajectory(white2boardpart2leftPerpendicular);
            sleep(200);
            wBot.openLeftClaw();
            wBot.openRightClaw();
            sleep(200);
            //drive.followTrajectory(strafe18R);
            //returning
            //drive.followTrajectory(return2midSetLeft);
            wBot.slideToTarget(0,-1);
            //drive.followTrajectory(return2midLeft);
            drive.followTrajectory(return2midLeftP);



            wBot.setSlideAngle(0.014);//0.053// pickup angle

            wBot.clawVAngle.setPosition(0.3);
            wBot.clawHAngle.setPosition(0.5);

            drive.followTrajectory(return2whites);
            wBot.slideToTarget(750,1);
            //wBot.clawHAngle.setPosition(0.53);
            sleep(400);
            //wBot.clawHAngle.setPosition(0.56);
            wBot.closeRightClaw();
            wBot.closeLeftClaw();
            sleep(100);
            //sleep(400);
            drive.followTrajectory(back79);
            wBot.clawVAngle.setPosition(0.2);
            wBot.clawHAngle.setPosition(0.37);

            //Placement again
            drive.followTrajectory(white2boardpart2Mid);



            wBot.openRightClaw();
            wBot.openLeftClaw();





            sleep(300);
            wBot.slideToTarget(0,-1);
            wBot.resetCLaw4Park();
            wBot.clawHAngle.setPosition(0.5);

            //drive.followTrajectory(leftParkPrepare);
            //drive.followTrajectory(parkEnding);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);






//_______2222222222-2-2-2-22-2-2-2-2-2-2-2-2-22-2-2-2-22-2-2-2-22-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2
        } else if (zoneDetected == 2) {
            wBot.clawVAngle.setPosition(0.35);
            wBot.setSlideAngle(0.2);

            drive.followTrajectory(start2midspike);
            //sleep(100);

            wBot.openLeftClaw();//place
            // to white pixels, turned towards spike
            //sleep(100);
            drive.followTrajectory(spike2whitemid);
            sleep(100);
            //clawVAngle.setPosition(0.28);
            wBot.clawVAngle.setPosition(0.34);
            //sleep(400);
            wBot.slideToTarget(750,1);
            wBot.setSlideAngle(0.103);
            //wBot.setSlideAngle(0.08);
            sleep(500);
            wBot.closeLeftClaw(); // +1, hopefully
            sleep(300);


            drive.followTrajectory(back79);

            wBot.clawVAngle.setPosition(0.13);
            //wBot.clawHAngle.setPosition(0.25);
            //drive.followTrajectory(white2boardpart2left);
            drive.followTrajectory(white2boardpart2midPerpendicular);
            sleep(200);
            wBot.openLeftClaw();
            wBot.openRightClaw();
            sleep(200);
            //drive.followTrajectory(strafe18R);
            //returning
            //drive.followTrajectory(return2midSetLeft);
            wBot.slideToTarget(0,-1);
            //drive.followTrajectory(return2midLeft);
            drive.followTrajectory(return2midMid);



            wBot.setSlideAngle(0.014);//0.053// pickup angle

            wBot.clawVAngle.setPosition(0.3);
            wBot.clawHAngle.setPosition(0.5);

            drive.followTrajectory(return2whites);
            wBot.slideToTarget(750,1);
            wBot.clawHAngle.setPosition(0.53);
            sleep(400);
            wBot.clawHAngle.setPosition(0.56);
            wBot.closeRightClaw();
            wBot.closeLeftClaw();
            sleep(100);
            //sleep(400);
            drive.followTrajectory(back79);
            wBot.clawVAngle.setPosition(0.2);
            wBot.clawHAngle.setPosition(0.63);

            //Placement again
            drive.followTrajectory(white2boardpart2Right);

            sleep(100);

            wBot.openRightClaw();
            wBot.openLeftClaw();
            sleep(300);

            drive.turn(Math.toRadians(5));
            wBot.clawHAngle.setPosition(0.355);



            sleep(400);
            wBot.slideToTarget(0,-1);
            wBot.resetCLaw4Park();
            wBot.clawHAngle.setPosition(0.5);

/*
            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.35);
            slide.setTargetPosition(500);
            drive.followTrajectory(start2midspike);
            sleep(230);





            clawHAngle.setPosition(0.4);
            clawR.setPosition(0.5); //place
            clawHAngle.setPosition(hPos);



            slideRAngle.setPosition(0.3);
            slideLAngle.setPosition(0.3);

            drive.followTrajectory(spike2whitemid);
            slideLAngle.setPosition(0.17);
            slideRAngle.setPosition(0.17);
            sleep(200);
            slideLAngle.setPosition(0.07);
            slideRAngle.setPosition(0.07);
            //clawVAngle.setPosition(0.28);
            clawVAngle.setPosition(0.35);
            sleep(2000);
            clawR.setPosition(0); // +1, hopefully


            drive.followTrajectory(back79);
            drive.followTrajectory(white2boardpart2);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawHAngle.setPosition(0.38);
            clawVAngle.setPosition(0.2);
            slide.setTargetPosition(700);
            slide.setPower(1);


            drive.turn(Math.toRadians(10));

            //drive.followTrajectory(rightplace);


            sleep(300);

            clawL.setPosition(0.5);
            clawR.setPosition(0.5);


            sleep(200);
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(200);
            drive.followTrajectory(right2boardmid);

            drive.followTrajectory(strafe2midR);
            clawL.setPosition(0);
            clawR.setPosition(0);

            drive.followTrajectory(back11);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);

            clawHAngle.setPosition(0.5);
            clawVAngle.setPosition(1);

 */


//___333333333333333333333333333333333333333333333333333333333333333333333333
        } else if (zoneDetected == 1) {
            wBot.clawVAngle.setPosition(0.35);
            wBot.setSlideAngle(0.2);

            drive.followTrajectory(start2rightspike);
            //sleep(100);

            wBot.openRightClaw();//place
            // to white pixels, turned towards spike
            //sleep(100);
            drive.followTrajectory(spike2whiteright);
            sleep(100);
            //clawVAngle.setPosition(0.28);
            wBot.clawVAngle.setPosition(0.34);
            //sleep(400);
            wBot.slideToTarget(750,1);
            wBot.setSlideAngle(0.103);
            //wBot.setSlideAngle(0.08);
            sleep(500);
            wBot.closeRightClaw(); // +1, hopefully
            sleep(300);


            drive.followTrajectory(back79);

            wBot.clawVAngle.setPosition(0.13);
            //wBot.clawHAngle.setPosition(0.25);
            //drive.followTrajectory(white2boardpart2left);
            drive.followTrajectory(white2boardpart2rightPerpendicular);
            sleep(200);
            wBot.openLeftClaw();
            wBot.openRightClaw();
            sleep(200);
            //drive.followTrajectory(strafe18R);
            //returning
            //drive.followTrajectory(return2midSetLeft);
            wBot.slideToTarget(0,-1);
            //drive.followTrajectory(return2midLeft);
            drive.followTrajectory(return2midRight);



            wBot.setSlideAngle(0.014);//0.053// pickup angle

            wBot.clawVAngle.setPosition(0.3);
            wBot.clawHAngle.setPosition(0.5);

            drive.followTrajectory(return2whites);
            wBot.slideToTarget(750,1);
            wBot.clawHAngle.setPosition(0.53);
            sleep(400);
            wBot.clawHAngle.setPosition(0.56);
            wBot.closeRightClaw();
            wBot.closeLeftClaw();
            sleep(100);
            //sleep(400);
            drive.followTrajectory(back79);
            wBot.clawVAngle.setPosition(0.2);
            wBot.clawHAngle.setPosition(0.37);

            //Placement again
            drive.followTrajectory(white2boardpart2Mid);

            sleep(100);

            wBot.openRightClaw();
            wBot.openLeftClaw();
            sleep(300);

            drive.turn(Math.toRadians(5));
            wBot.clawHAngle.setPosition(0.355);



            sleep(400);
            wBot.slideToTarget(0,-1);
            wBot.resetCLaw4Park();
            wBot.clawHAngle.setPosition(0.5);
            /*
            slideLAngle.setPosition(0.45);
            slideRAngle.setPosition(0.45);

            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.35);
            drive.followTrajectory(start2rightspike);
            sleep(230);


            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(225);
            slide.setPower(0);

            sleep(500);
            clawR.setPosition(0.5); //place



            slideRAngle.setPosition(0.3);
            slideLAngle.setPosition(0.3);

            drive.followTrajectory(spike2whiteright);
            slideLAngle.setPosition(0.20);
            slideRAngle.setPosition(0.20);
            clawVAngle.setPosition(0.5);
            clawR.setPosition(0); // +1, hopefully


            drive.followTrajectory(back79);
            drive.followTrajectory(white2boardpart2);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawHAngle.setPosition(0.27);
            clawVAngle.setPosition(0.2);
            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(320);

            slide.setPower(0);
            drive.followTrajectory(leftplace);



            sleep(300);

            clawL.setPosition(0.5);
            clawR.setPosition(0.5);


            sleep(200);
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(200);
            drive.followTrajectory(left2boardmid);
            drive.followTrajectory(strafe2midR);
            clawL.setPosition(0);
            clawR.setPosition(0);

            drive.followTrajectory(back11);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);

            clawHAngle.setPosition(0.5);
            clawVAngle.setPosition(1);

             */

        }








    }
}
