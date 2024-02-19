package org.firstinspires.ftc.teamcode.drive.autonRoutesOld;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "R_Far_Door_Edge")

public class R_Far_Door_Edge extends LinearOpMode {


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







        Pose2d startPose = new Pose2d(-34.50, -62.84, Math.toRadians(90.00));

        drive.setPoseEstimate(startPose);
        Trajectory start2whiteleft = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-34.5, -30, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-46,-11.5, Math.toRadians(270)))
                .build();
        Trajectory back79 = drive.trajectoryBuilder(new Pose2d(-46,-11.5, Math.toRadians(180)))
                .back(79)
                .build();
        Trajectory white2boardpart2 = drive.trajectoryBuilder(back79.end())
                .lineToSplineHeading(new Pose2d(48, -16, Math.toRadians(-45)))
                .build();

        Trajectory start2midspike = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-56, -23, Math.toRadians(0.00)))
                .build();
        Trajectory spike2whitemid = drive.trajectoryBuilder(start2midspike.end())
                .lineToSplineHeading(new Pose2d(-46, -11.5, Math.toRadians(180.00)))
                .build();

        Trajectory start2rightspike = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-48, -30, Math.toRadians(0.00)))
                .build();
        Trajectory spike2whiteright = drive.trajectoryBuilder(start2rightspike.end())
                .lineToSplineHeading(new Pose2d(-46, -11.5, Math.toRadians(180.00)))
                .build();



        Trajectory leftplace = drive.trajectoryBuilder(white2boardpart2.end())
                .strafeLeft(6)
                .build();
        Trajectory left2boardmid = drive.trajectoryBuilder(leftplace.end())
                .strafeRight(6)
                .build();
        Trajectory rightplace = drive.trajectoryBuilder(white2boardpart2.end())
                .strafeRight(7)
                .build();
        Trajectory right2boardmid = drive.trajectoryBuilder(rightplace.end())
                .strafeLeft(7)
                .build();



        Trajectory strafe18R = drive.trajectoryBuilder(white2boardpart2.end())
                .strafeRight(18)
                .build();
        Trajectory strafe18L = drive.trajectoryBuilder(strafe18R.end())
                .strafeLeft(18)
                .build();



        //Spline Trajectories

        Trajectory strafe2midR = drive.trajectoryBuilder(white2boardpart2.end())
                .lineToSplineHeading(new Pose2d(48, -10, Math.toRadians(180)))
                .build();
        Trajectory back20 = drive.trajectoryBuilder(strafe2midR.end())
                .back(11)
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

        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        int zoneDetectionPing1 = 0;
        int zoneDetectionPing2 = 0;
        int zoneDetectionPing3 = 0;
        int zoneDetected = 0;
        while (opModeInInit() && StopSearch == 0) {

            if (!OpenCVDetectTeamProp.isDetected) {
                zoneDetected = 3;
                zoneDetectionPing3++;

            } else if (OpenCVDetectTeamProp.centerX < 200) {
                zoneDetected = 1;
                zoneDetectionPing1++;


            } else if (OpenCVDetectTeamProp.centerX >= 200) {
                zoneDetected = 2;
                zoneDetectionPing2++;

            } else if (zoneDetectionPing1 > 10 || zoneDetectionPing2 > 10 || zoneDetectionPing3 > 10) {
                StopSearch = 1;
            }
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
            slide.setTargetPosition(1);
            sleep(500);
            slide.setTargetPosition(0);
            sleep(500);

            slideLAngle.setPosition(0.45);
            slideRAngle.setPosition(0.45);

            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.35);
            drive.followTrajectory(start2whiteleft); // to white pixels, turned towards spike

            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(225);
            slide.setPower(0);

            sleep(500);
            clawR.setPosition(0.5); //place


            slideRAngle.setPosition(0.3);
            slideLAngle.setPosition(0.3);

            drive.turn(Math.toRadians(-90));
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

            drive.followTrajectory(back20);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);

            clawHAngle.setPosition(0.5);
            clawVAngle.setPosition(1);

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
            slideLAngle.setPosition(0.45);
            slideRAngle.setPosition(0.45);


            slideLAngle.setPosition(0.10);
            slideRAngle.setPosition(0.10);
            clawVAngle.setPosition(0.35);
            drive.followTrajectory(start2midspike);
            sleep(230);


            slidePower = getSlideVelocity(1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(225);
            slide.setPower(0);

            sleep(500);
            clawR.setPosition(0.5); //place
            sleep(500);


            slideRAngle.setPosition(0.3);
            slideLAngle.setPosition(0.3);

            drive.followTrajectory(spike2whitemid);
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

            drive.followTrajectory(rightplace);


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

            drive.followTrajectory(back20);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);

            clawHAngle.setPosition(0.5);
            clawVAngle.setPosition(1);


//___333333333333333333333333333333333333333333333333333333333333333333333333
        } else if (zoneDetected == 3) {
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

            drive.followTrajectory(strafe18R);


            sleep(300);

            clawL.setPosition(0.5);
            clawR.setPosition(0.5);


            sleep(200);
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(sudoTriggerDepth, 3));
            slide.setPower(slidePower);
            sleep(200);
            drive.followTrajectory(strafe18L);
            drive.followTrajectory(strafe2midR);
            clawL.setPosition(0);
            clawR.setPosition(0);

            drive.followTrajectory(back20);
            //drive.followTrajectory(return2sender);
            //drive.followTrajectory(return2sender2);

            clawHAngle.setPosition(0.5);
            clawVAngle.setPosition(1);

        }








    }
}
