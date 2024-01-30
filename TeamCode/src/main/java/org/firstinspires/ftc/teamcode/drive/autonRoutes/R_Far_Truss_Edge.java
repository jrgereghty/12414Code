package org.firstinspires.ftc.teamcode.drive.autonRoutes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVGreatestColorTest;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "R_Far_Truss_Edge")

public class R_Far_Truss_Edge extends LinearOpMode {


    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    //PointFollower follower = new PointFollower(this);
    //public static boolean isTest = false;
    public static boolean isParkFinal = true;



    double slidePos = 0.0;







    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
                .strafeRight(2)
                .build();
        Trajectory strafe2left = drive.trajectoryBuilder(strafe2right.end())
                .strafeLeft(2)
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
        Trajectory back10 = drive.trajectoryBuilder(start2board.end())
                .back(10)
                .build();
        /*
        Trajectory strafe2midR = drive.trajectoryBuilder(back11.end())
                .lineToSplineHeading(new Pose2d(39, -10, Math.toRadians(180)))
                .build();



        Trajectory return2sender = drive.trajectoryBuilder(back20.end())
                .forward(95)
                .build();
        Trajectory return2sender2 = drive.trajectoryBuilder(return2sender.end())
                .lineToSplineHeading(startPose)
                .build();

         */
        Trajectory strafe2edgeR = drive.trajectoryBuilder(back10.end())
                .lineToSplineHeading(new Pose2d(39, -59, Math.toRadians(180)))
                .build();
        Trajectory back20 = drive.trajectoryBuilder(strafe2edgeR.end())
                .back(25)
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




        slide = hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(1);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLAngle = hardwareMap.servo.get("slideLAngle");
        slideLAngle.setDirection(Servo.Direction.REVERSE);
        slideLAngle.setPosition(0.8);

        slideRAngle = hardwareMap.servo.get("slideRAngle");
        slideRAngle.setPosition(0.8);

        clawHAngle = hardwareMap.servo.get("clawHAngle");
        clawHAngle.scaleRange(0.04, 1);
        clawHAngle.setPosition(0.5);

        clawVAngle = hardwareMap.servo.get("clawVAngle");
        clawVAngle.scaleRange(0, 0.7);
        clawVAngle.setPosition(1);

        clawL = hardwareMap.servo.get("clawL");
        clawL.scaleRange(0.21, 0.605);
        clawL.setPosition(0);

        clawR = hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
        clawR.scaleRange(0.20, 0.595);
        clawR.setPosition(0);

        int StopSearch = 0;
        int zoneDetectionPing1 = 0;
        int zoneDetectionPing2 = 0;
        int zoneDetectionPing3 = 0;
        int zoneDetected = 0;

        while (opModeInInit() && StopSearch == 0) {
            if (!OpenCVDetectTeamProp.isDetected) {
                zoneDetectionPing3++;
            } else if (OpenCVDetectTeamProp.centerX < 200) {
                zoneDetectionPing1++;
            } else if (OpenCVDetectTeamProp.centerX >= 200) {
                zoneDetectionPing2++;
            }
            if (zoneDetectionPing1 >= 100 || zoneDetectionPing2 >= 100 || zoneDetectionPing3 >= 100) {
                if (zoneDetectionPing1 >= 100) {
                    zoneDetected = 1;
                } else if (zoneDetectionPing2 >= 100) {
                    zoneDetected = 2;
                } else if (zoneDetectionPing3 >= 100) {
                    zoneDetected = 3;
                }
                StopSearch = 1;
            }
        }
        telemetry.addData("zoneDetected", zoneDetected);
        telemetry.update();
        waitForStart();

        slidePos = slide.getCurrentPosition() / 537.7 * 4 * Math.PI / 54.864;

        drive.followTrajectory(forward10);


        /*
        if (zoneDetected == 1) {


            slideLAngle.setPosition(0.45);
            slideRAngle.setPosition(0.45);

            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.35);
            sleep(250);
            drive.followTrajectory(line4start);
            sleep(230);




            sleep(225);
            slide.setPower(0);



            sleep(500);
            clawR.setPosition(0.5);



            sleep(500);



            sleep(800);
            drive.followTrajectory(pixel2start);
            drive.followTrajectory(forward60);
            sleep(200);

            drive.followTrajectory(start2board);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawVAngle.setPosition(0.2);

            sleep(320);

            slide.setPower(0);

            drive.followTrajectory(leftplace);


            sleep(300);

            clawL.setPosition(0.5);


            sleep(200);

            sleep(200);
            drive.followTrajectory(left2boardmid);
            drive.followTrajectory(back11);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2edgeR);

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





//_______2222222222-2-2-2-22-2-2-2-2-2-2-2-2-22-2-2-2-22-2-2-2-22-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2-2
        } else if (zoneDetected == 2) {
            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.40);
            sleep(180);

            drive.followTrajectory(line4startmid);
            sleep(180);






            sleep(377);
            slide.setPower(0);


            sleep(500);
            clawR.setPosition(0.5);



            sleep(300);



            sleep(1000);
            drive.followTrajectory(pixel2startmid);

            drive.followTrajectory(forward60);
            sleep(200);

            drive.followTrajectory(start2board);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawVAngle.setPosition(0.15);

            sleep(320);
            clawVAngle.setPosition(0.25);
            slide.setPower(0);

            sleep(300);

            clawL.setPosition(0.5);

            sleep(200);
            drive.followTrajectory(strafe2left);

            sleep(260);
            drive.followTrajectory(strafe2right);
            drive.followTrajectory(back11);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2edgeR);

            drive.followTrajectory(back20);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);
            clawVAngle.setPosition(1);
            sleep(20000);












//___333333333333333333333333333333333333333333333333333333333333333333333333
        } else if (zoneDetected == 3) {


            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.4);




            drive.followTrajectory(line4startright);
            sleep(200);



            sleep(230);
            slide.setPower(0);


            sleep(500);
            clawR.setPosition(0.5);



            sleep(500);



            sleep(1000);
            drive.followTrajectory(pixel2startright);
            drive.followTrajectory(forward60);
            slideLAngle.setPosition(0.4);
            slideRAngle.setPosition(0.4);
            clawVAngle.setPosition(0.2);

            drive.followTrajectory(start2board);


            sleep(270);


            slide.setPower(0);
            clawVAngle.setPosition(0.25);
            drive.followTrajectory(rightplace);

            sleep(300);

            clawL.setPosition(0.5);


            sleep(200);

            sleep(300);
            drive.followTrajectory(right2boardmid);
            drive.followTrajectory(back11);
            clawVAngle.setPosition(1);
            drive.followTrajectory(strafe2edgeR);

            drive.followTrajectory(back20);

            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);
            clawVAngle.setPosition(1);
            sleep(20000);

        }*/








    }
}
