package org.firstinspires.ftc.teamcode.drive.autonRoutesOld;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVGreatestColorTest;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "R_Clo_Truss_Edge_REAL")

public class R_Clo_Truss_Edge_REAL extends LinearOpMode {


    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    //PointFollower follower = new PointFollower(this);
    //public static boolean isTest = false;
    public static boolean isParkFinal = true;

    public static double hPos = 0.5;
    public static double vPos = 1;
    public static double pos = 0.8;

    double slideLength = 0.0;
    double slidePos = 0.0;
    public static int[] detections = new int[20];
    double slidePower;
    double startDelay = 0;
    double sudoTrigger;
    double sudoTriggerDepth = 1;

    private static double getSlideVelocity(int sudoTrigger, double slidePos, double sudoTriggerDepth) {
        double velocity = 0.0;
        if (sudoTrigger == 1) {
            velocity = sudoTriggerDepth * (0.6 * Math.cos(0.5 * Math.PI * slidePos) + 0.4);
        } else if (sudoTrigger == -1) {
            velocity = sudoTriggerDepth * (-0.6 * Math.sin(0.5 * Math.PI * slidePos) - 0.4);
        }
        return(velocity);
    }



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        DcMotor slide;
        Servo clawHAngle;
        Servo clawVAngle;
        Servo slideLAngle;
        Servo slideRAngle;
        Servo clawL;
        Servo clawR;

        slide = hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        slideLAngle = hardwareMap.servo.get("slideLAngle");
        slideLAngle.setDirection(Servo.Direction.REVERSE);
        slideLAngle.setPosition(pos);

        slideRAngle = hardwareMap.servo.get("slideRAngle");
        slideRAngle.setPosition(pos);

        clawHAngle = hardwareMap.servo.get("clawHAngle");
        clawHAngle.scaleRange(0.04, 1);
        clawHAngle.setPosition(hPos);

        clawVAngle = hardwareMap.servo.get("clawVAngle");
        clawVAngle.scaleRange(0, 0.7);
        clawVAngle.setPosition(vPos);



        clawL = hardwareMap.servo.get("clawL");
        clawL.scaleRange(0.21, 0.605);
        clawL.setPosition(0.0);

        clawR = hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
        clawR.scaleRange(0.20, 0.595);
        clawR.setPosition(0.0);







        Pose2d startPose = new Pose2d(12, -63.5, Math.toRadians(90.00));

        drive.setPoseEstimate(startPose);
        Trajectory forward30 = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();
        Trajectory line4startright = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, -53.36, Math.toRadians(60)))//18
                .addTemporalMarker(0.5, ()-> {
                    slide.setPower(1);
                    slide.setTargetPosition(600);

                })
                .build();

        Trajectory line4startmid = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, -51.36, Math.toRadians(90)))
                .build();
        Trajectory line4start = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(12, -53.36, Math.toRadians(127)))//37
                .addTemporalMarker(0.5, ()-> {
                    slide.setPower(1);
                    slide.setTargetPosition(600);

                })
                .build();
        Trajectory pixel2startright = drive.trajectoryBuilder(line4start.end())
                .lineToSplineHeading(new Pose2d(12, -58.6, Math.toRadians(0)))
                .build();
        Trajectory pixel2startmid = drive.trajectoryBuilder(line4startmid.end())
                .lineToSplineHeading(new Pose2d(12, -58.6, Math.toRadians(0)))
                .build();
        Trajectory pixel2start = drive.trajectoryBuilder(line4startright.end())
                .lineToSplineHeading(new Pose2d(12, -58.6, Math.toRadians(0)))
                .build();
        /*
        Trajectory forward60 = drive.trajectoryBuilder(new Pose2d(-43, -58.6, Math.toRadians(0)))
                .lineTo(new Vector2d(24, -59.8))
                .build();

         */
        Trajectory start2board = drive.trajectoryBuilder(new Pose2d(43, -58.6, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(46.01, -36.5), Math.toRadians(0.00))//46.01, 36
                .build();
        Trajectory leftplace = drive.trajectoryBuilder(start2board.end())
                .strafeLeft(6)
                .build();
        Trajectory left2boardmid = drive.trajectoryBuilder(start2board.end())
                .strafeRight(6)
                .build();
        Trajectory rightplace = drive.trajectoryBuilder(start2board.end())
                .strafeRight(7)
                .build();
        Trajectory right2boardmid = drive.trajectoryBuilder(start2board.end())
                .strafeLeft(7)
                .build();




        Trajectory forward10 = drive.trajectoryBuilder(startPose)
                .forward(10)
                .addTemporalMarker(2, ()->{
                    slide.setTargetPosition(2);
                })

                .build();
        Trajectory strafe18 = drive.trajectoryBuilder(startPose)
                .strafeRight(18)
                .build();
        Trajectory forward5 = drive.trajectoryBuilder(startPose)
                .forward(5)
                .build();

        Trajectory forward25 = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();
        Trajectory forward40 = drive.trajectoryBuilder(startPose)
                .forward(40)
                .build();
        Trajectory line90 = drive.trajectoryBuilder(start2board.end())
                .lineToSplineHeading(new Pose2d(39.67, -42.47, Math.toRadians(270.00)))
                .build();
        //Spline Trajectories
        Trajectory board2truss = drive.trajectoryBuilder(line90.end())
                .splineTo(new Vector2d(-17.73, -58.62), Math.toRadians(180.00))
                .splineTo(new Vector2d(-49.57, -46.43), Math.toRadians(212.07))

                .build();
        Trajectory back11 = drive.trajectoryBuilder(start2board.end())
                .back(11)
                .build();
        Trajectory strafe2edgeR = drive.trajectoryBuilder(back11.end())
                .lineToSplineHeading(new Pose2d(39, -61, Math.toRadians(180)))
                .build();
        Trajectory back20 = drive.trajectoryBuilder(strafe2edgeR.end())
                .back(20)
                .build();

        Trajectory return2sender = drive.trajectoryBuilder(back20.end())
                .forward(95)
                .build();
        Trajectory return2sender2 = drive.trajectoryBuilder(new Pose2d(12, -58.6, Math.toRadians(0)))
                .lineToSplineHeading(startPose)
                .build();















        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        pipeline = new OpenCVGreatestColorTest(telemetry);
        //webcam2.setPipeline(pipeline);
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




        //___________________________________________________________________




        //OLD SPOT FOR HARDWARE MAPPING


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
            if(Zone1detections >= Zone2detections && Zone1detections >= Zone3detections){zoneDetected=1;}
            else if(Zone2detections >= Zone1detections && Zone2detections >= Zone3detections){zoneDetected=2;}
            else if(Zone3detections >= Zone1detections && Zone3detections >= Zone2detections){zoneDetected=3;}


        }
        waitForStart();

        slidePos = slide.getCurrentPosition() / 537.7 * 4 * Math.PI / 54.864;
        slideLength = slidePos * 54.864 + 38.5;

        /*
        } else if (gamepad1.left_trigger > 0.01) {
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(gamepad1.left_trigger, 3));
            slide.setPower(slidePower);
        } else {
            slide.setPower(0);
        }

         */



        if (zoneDetected == 1) {

            clawVAngle.setPosition(0.4);
            sleep(300);




            slideLAngle.setPosition(0.2);
            slideRAngle.setPosition(0.2);

            drive.followTrajectory(line4start);
            sleep(230);


            clawR.setPosition(0.5);



            sleep(200);
            slide.setTargetPosition(0);
            slide.setPower(-1);
            slideLAngle.setPosition(0.25);
            slideRAngle.setPosition(0.25);
            sleep(600);
            slideLAngle.setPosition(0.25);
            slideRAngle.setPosition(0.25);
            clawVAngle.setPosition(0.2);
            drive.followTrajectory(pixel2start);



            sleep(200);


            drive.followTrajectory(start2board);

            sleep(320);


            drive.followTrajectory(leftplace);
            slide.setTargetPosition(550);
            slide.setPower(1);

            sleep(400);

            clawL.setPosition(0.5);


            sleep(200);
            slide.setTargetPosition(0);
            slide.setPower(-1);
            sleep(200);
            drive.followTrajectory(left2boardmid);
            drive.followTrajectory(back11);
            clawL.setPosition(0);
            clawR.setPosition(0);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2edgeR);

            drive.followTrajectory(back20);

            slideLAngle.setPosition(0.35);
            slideRAngle.setPosition(0.35);
            clawVAngle.setPosition(1);

            //drive.followTrajectory(return2sender);


            //drive.followTrajectory(return2sender2);



            sleep(20000);
/*

            drive.turn(Math.toRadians(90));
            sleep(1000);
            drive.followTrajectory(board2truss);


            sleep(2000);

            drive.turn(Math.toRadians(20));
            slideLAngle.setPosition(0.25);
            slideRAngle.setPosition(0.25);
            clawR.setPosition(0);
            clawL.setPosition(0);
            drive.turn(Math.toRadians(150));
            drive.followTrajectory(pixel2start);



            drive.followTrajectory(start2board);
            drive.followTrajectory(back11);

             */



//_______2222222222-2-2-2-22-2-2-2-2-2-2-2-2-22-2-2-2-22-2-2-2-22-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2
        } else if (zoneDetected == 2) {

            slideLAngle.setPosition(0.20);
            slideRAngle.setPosition(0.20);
            clawVAngle.setPosition(0.40);
            sleep(300);
            slide.setTargetPosition(900);
            slide.setPower(1);
            drive.followTrajectory(line4startmid);
            sleep(200);
            clawR.setPosition(0.5);
            slide.setTargetPosition(0);
            slide.setPower(-1);
            sleep(300);

            drive.followTrajectory(pixel2startmid);

            //drive.followTrajectory(forward60);
            slideLAngle.setPosition(0.35);
            slideRAngle.setPosition(0.35);
            clawVAngle.setPosition(0.15);
            drive.followTrajectory(start2board);


            slide.setTargetPosition(700);
            slide.setPower(1);


            sleep(300);
            clawL.setPosition(0.5);
            sleep(200);



            slide.setTargetPosition(0);
            slide.setPower(-1);
            sleep(400);
            drive.followTrajectory(back11);
            clawL.setPosition(0);
            clawR.setPosition(0);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2edgeR);

            drive.followTrajectory(back20);
            clawL.setPosition(0);
            clawR.setPosition(0);

            //drive.followTrajectory(return2sender);



            //drive.followTrajectory(return2sender2);


            clawVAngle.setPosition(1);
            sleep(20000);












//___333333333333333333333333333333333333333333333333333333333333333333333333
        } else if (zoneDetected == 3) {

            clawVAngle.setPosition(0.4);
            slideLAngle.setPosition(0.15);
            slideRAngle.setPosition(0.15);
            slide.setTargetPosition(600);
            slide.setPower(1);
            drive.followTrajectory(line4startright);
            sleep(200);

            clawR.setPosition(0.5);

            sleep(300);

            slide.setTargetPosition(0);
            slide.setPower(-1);

            sleep(1000);

            slideLAngle.setPosition(0.33);
            slideRAngle.setPosition(0.33);
            clawVAngle.setPosition(0.14);
            drive.followTrajectory(pixel2startright);

            //drive.followTrajectory(forward60);


            drive.followTrajectory(start2board);

            sleep(290);



            drive.followTrajectory(rightplace);
            slide.setTargetPosition(600);
            slide.setPower(1);
            sleep(700);

            clawL.setPosition(0.5);


            sleep(200);
            slide.setTargetPosition(0);
            slide.setPower(-1);
            sleep(300);
            drive.followTrajectory(right2boardmid);
            drive.followTrajectory(back11);
            clawL.setPosition(0);
            clawR.setPosition(0);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2edgeR);
            clawVAngle.setPosition(1);
            clawL.setPosition(0);
            clawR.setPosition(0);
            drive.followTrajectory(back20);



            //drive.followTrajectory(return2sender);
            //          d   rive.followTrajectory(return2sender2);



            sleep(30000);

        }








    }
}

