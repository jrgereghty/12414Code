package org.firstinspires.ftc.teamcode.drive;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LevineLocal.PointFollower;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedAuto")

public class TomatoAuton extends LinearOpMode {


    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;
    PointFollower follower = new PointFollower(this);
    public static boolean isTest = false;
    public static boolean isParkFinal = true;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        DcMotor slideLeft;
        DcMotor slideRight;

        DcMotor slurp;


        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);
        Trajectory forward30 = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();
        Trajectory forward10 = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();
        Trajectory back11 = drive.trajectoryBuilder(startPose)
                .back(11)
                .build();
        Trajectory forward25 = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();
        Trajectory forward40 = drive.trajectoryBuilder(startPose)
                .forward(40)
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




        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);


        slurp = hardwareMap.dcMotor.get("slurp");
        slurp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slurp.setDirection(DcMotor.Direction.REVERSE);


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



            if (zoneDetected == 1) {
                drive.followTrajectory(forward40);
                drive.turn(Math.toRadians(100));
                drive.followTrajectory(back11);

                slurp.setPower(-0.6);
                sleep(1000);
                drive.followTrajectory(forward30);
                drive.followTrajectory(forward25);

            } else if (zoneDetected == 2) {
                drive.followTrajectory(forward30);
                slurp.setPower(-0.6);
                sleep(1000);


                drive.turn(Math.toRadians(100));
                drive.followTrajectory(forward30);
                drive.followTrajectory(forward10);


            } else if (zoneDetected == 3) {
                drive.followTrajectory(forward40);
                drive.turn(Math.toRadians(-100));
                slurp.setPower(-0.6);
                sleep(600);
                drive.turn(Math.toRadians(200));
                drive.followTrajectory(forward40);
                drive.followTrajectory(forward10);

            }





        //middle spike
        /*drive.followTrajectory(forward30);
        slurp.setPower(-0.6);
        sleep(1000);


        drive.turn(Math.toRadians(100));
        drive.followTrajectory(forward30);
        drive.followTrajectory(forward10);/*

         */
        //left spike

        /*drive.followTrajectory(forward40);
        drive.turn(Math.toRadians(100));
        drive.followTrajectory(back11);

        slurp.setPower(-0.6);
        sleep(1000);
        drive.followTrajectory(forward30);
        drive.followTrajectory(forward25);

        //right spike
        /*
        drive.followTrajectory(forward40);
        drive.turn(Math.toRadians(-100));
        slurp.setPower(-0.6);
        sleep(600);
        drive.turn(Math.toRadians(200));
        drive.followTrajectory(forward40);
        drive.followTrajectory(forward10);
         */

    }
}























