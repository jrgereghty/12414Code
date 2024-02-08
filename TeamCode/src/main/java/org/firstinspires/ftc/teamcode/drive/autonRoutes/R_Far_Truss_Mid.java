package org.firstinspires.ftc.teamcode.drive.autonRoutes;

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

@Autonomous(name = "R_Far_Truss_Mid")

public class R_Far_Truss_Mid extends LinearOpMode {


    OpenCvCamera webcam;
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
    double sudoTrigger;
    double sudoTriggerDepth = 1;
    public static int[] detections = new int[20];

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







        Pose2d startPose = new Pose2d(-36.00, -62.84, Math.toRadians(90.00));

        drive.setPoseEstimate(startPose);
        Trajectory forward30 = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();
        Trajectory line4start = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, -53.36, Math.toRadians(115.00))) //312
                .build();
        Trajectory line4startmid = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, -51.36, Math.toRadians(90)))
                .build();
        Trajectory line4startright = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-36, -53.36, Math.toRadians(60)))
                .build();
        Trajectory pixel2start = drive.trajectoryBuilder(line4start.end())
                .lineToSplineHeading(new Pose2d(-36, -59.3, Math.toRadians(0)))
                .build();
        Trajectory pixel2startmid = drive.trajectoryBuilder(line4startmid.end())
                .lineToSplineHeading(new Pose2d(-36, -59.3, Math.toRadians(0)))
                .build();
        Trajectory pixel2startright = drive.trajectoryBuilder(line4startright.end())
                .lineToSplineHeading(new Pose2d(-36, -59.3, Math.toRadians(0)))
                .build();
        Trajectory forward60 = drive.trajectoryBuilder(new Pose2d(-36, -59.3, Math.toRadians(0)))
                .forward(60)
                .build();
        Trajectory start2board = drive.trajectoryBuilder(forward60.end()) // START TO BOARD HERE
                .splineToConstantHeading(new Vector2d(48.01, -38), Math.toRadians(0.00))//46.01, 36
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

                .build();
        Trajectory strafe18 = drive.trajectoryBuilder(startPose)
                .strafeRight(18)
                .build();
        Trajectory forward5 = drive.trajectoryBuilder(startPose)
                .forward(5)
                .build();
        Trajectory strafe2right = drive.trajectoryBuilder(start2board.end())
                .strafeRight(19)
                .build();
        Trajectory strafe2left = drive.trajectoryBuilder(strafe2right.end())
                .strafeLeft(19)
                .build();

        Trajectory forward25 = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();
        Trajectory forward40 = drive.trajectoryBuilder(startPose)
                .forward(40)
                .build();
        Trajectory line90 = drive.trajectoryBuilder(start2board.end())
                .lineToSplineHeading(new Pose2d(39.67, -42.47, Math.toRadians(90.00)))
                .build();
        //Spline Trajectories
        Trajectory board2truss = drive.trajectoryBuilder(line90.end())
                .splineTo(new Vector2d(-17.73, -58.62), Math.toRadians(180.00))
                .splineTo(new Vector2d(-49.57, -46.43), Math.toRadians(212.07))

                .build();
        Trajectory back11 = drive.trajectoryBuilder(start2board.end())
                .back(5)
                .build();
        Trajectory strafe2midR = drive.trajectoryBuilder(back11.end())
                .lineToSplineHeading(new Pose2d(39, -10, Math.toRadians(180)))
                .build();
        Trajectory back20 = drive.trajectoryBuilder(strafe2midR.end())
                .back(20)
                .build();
        /*
        Trajectory return2sender = drive.trajectoryBuilder(back20.end())
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




        slide = hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLAngle = hardwareMap.servo.get("slideLAngle");
        slideLAngle.setDirection(Servo.Direction.REVERSE);
        slideLAngle.setPosition(0.8);

        slideRAngle = hardwareMap.servo.get("slideRAngle");
        slideRAngle.setPosition(0.8);

        clawHAngle = hardwareMap.servo.get("clawHAngle");
        clawHAngle.scaleRange(0.04, 1);
        clawHAngle.setPosition(hPos);

        clawVAngle = hardwareMap.servo.get("clawVAngle");
        clawVAngle.setPosition(vPos);
        clawVAngle.scaleRange(0, 0.6);


        clawL = hardwareMap.servo.get("clawL");
        clawL.scaleRange(0.21, 0.605);
        clawL.setPosition(lPos);

        clawR = hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
        clawR.scaleRange(0.17, 0.595);
        clawR.setPosition(rPos);



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
            sleep(5000);


            slideLAngle.setPosition(0.45);
            slideRAngle.setPosition(0.45);
            clawVAngle.setPosition(0.30);

            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);

            drive.followTrajectory(line4start);
            sleep(215);




            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(225);
            slide.setPower(0);



            sleep(500);
            clawR.setPosition(0.5);



            sleep(500);

            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);

            sleep(800);
            drive.followTrajectory(pixel2start);
            drive.followTrajectory(forward60);
            sleep(200);

            drive.followTrajectory(start2board);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawVAngle.setPosition(0.2);
            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(320);

            slide.setPower(0);

            drive.followTrajectory(leftplace);


            sleep(300);

            clawL.setPosition(0.5);


            sleep(200);
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(200);
            drive.followTrajectory(left2boardmid);
            drive.followTrajectory(back11);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2midR);

            drive.followTrajectory(back20);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);
            clawVAngle.setPosition(1);
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
            sleep(5000);
            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.2);

            drive.followTrajectory(line4startmid);
            sleep(180);

            clawVAngle.setPosition(0.40);



            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(377);
            slide.setPower(0);


            sleep(500);
            clawR.setPosition(0.5);



            sleep(300);

            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);

            sleep(1000);
            drive.followTrajectory(pixel2startmid);

            drive.followTrajectory(forward60);
            sleep(200);

            drive.followTrajectory(start2board);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawVAngle.setPosition(0.15);
            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(320);
            clawVAngle.setPosition(0.25);
            slide.setPower(0);

            sleep(300);

            clawL.setPosition(0.5);

            sleep(200);
            drive.followTrajectory(strafe2right);
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(260);
            drive.followTrajectory(strafe2left);
            clawVAngle.setPosition(1);
            drive.followTrajectory(back11);
            drive.followTrajectory(strafe2midR);

            drive.followTrajectory(back20);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);

            sleep(20000);












//___333333333333333333333333333333333333333333333333333333333333333333333333
        } else if (zoneDetected == 3) {
            sleep(5000);
            slideLAngle.setPosition(0.06);
            slideRAngle.setPosition(0.06);
            clawVAngle.setPosition(0.5);
            sleep(100);

            drive.followTrajectory(line4startright);
            sleep(200);



            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(200);
            slide.setPower(0);


            sleep(500);
            clawR.setPosition(0.5);



            sleep(500);

            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);

            sleep(1000);
            drive.followTrajectory(pixel2startright);
            drive.followTrajectory(forward60);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawVAngle.setPosition(0.2);

            drive.followTrajectory(start2board);

            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(270);


            slide.setPower(0);
            clawVAngle.setPosition(0.25);
            drive.followTrajectory(rightplace);

            sleep(300);

            clawL.setPosition(0.5);


            sleep(200);
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(300);
            drive.followTrajectory(right2boardmid);
            drive.followTrajectory(back11);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2midR);

            drive.followTrajectory(back20);
            clawVAngle.setPosition(1);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);

            sleep(20000);

        }








    }
}
