package org.firstinspires.ftc.teamcode.drive;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    //public static VisionPortal visionPortal;

    //public static WebcamName cam1;
    //public static AprilTagProcessor aprilTag;

    //VisionPortal myVisionPortal;

    OpenCvCamera webcam;
    static OpenCVDetectTeamProp colorPipe;
    static OpenCVGreatestColorTest pipeline;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //cam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        //CameraInit cameraPro = new CameraInit(hardwareMap);
        //myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"));

        //cameraPro.switchToFirstPipeline();
        //telemetry.addLine("Status: Initialized");

        DcMotor slideLeft;
        DcMotor slideRight;

        DcMotor slurp;



        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(30)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();

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





















        waitForStart(); //___________________________________________________________________

        if(isStopRequested()) return;



        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);

        //boxLid = hardwareMap.servo.get("boxLid");
        //boxLid.setPosition(0.5);

        slurp = hardwareMap.dcMotor.get("slurp");
        slurp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slurp.setDirection(DcMotor.Direction.REVERSE);


        //EpicSuperCoolAprilTags aprilTag = new EpicSuperCoolAprilTags();
        //aprilTag.cam1 = cam1;


        //myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraPro);
        //EpicSuperCoolAprilTags.initAprilTag();
        //aprilTag.runOpMode();
        //aprilTag.initAprilTag();

        //VisionPortal.Builder builder = new VisionPortal.Builder();
        //    builder.setCamera(cam1);
        //______________________________________________________________THIS IS A MARKER,   LINE
        // telemetry.addLine(cameraPro.getPipeline1Output());
        //telemetry.update();

        while (opModeInInit()) {
            if (!OpenCVDetectTeamProp.isDetected) {//idk why this is here, i assume just so it does SOMETHING
                zoneDetected = 1; // if it can't find a prop for some reason
            } else if (OpenCVDetectTeamProp.centerX < 81) {
                zoneDetected = 2;
            } else if (OpenCVDetectTeamProp.centerX > 559) {
                zoneDetected = 1;
            }
            /*

            if (zoneDetected == 1) {
                preloadPlacement = beacon1Preload;
            } else if (zoneDetected == 2) {
                preloadPlacement = beacon2Preload;
            } else {
                preloadPlacement = beacon3Preload;
            }
            telemetry.addData("preloadPlacement", preloadPlacement);
            telemetry.addLine("zoneDetected: " + zoneDetected);
            telemetry.update(); */
        }












        drive.followTrajectory(traj1);
        slurp.setPower(-0.6);
        sleep(1000);


        drive.turn(Math.toRadians(100));
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

/*
        List<AprilTagDetection> currentDetections = aprilTag.MythicalaprilTags();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == 1) {
                    //drive.turn(Math.toRadians(90));
                    drive.followTrajectory(Trajegy);
                }
            } // end of apriltags
        }
*/





        //drive.followTrajectory(Trajegy);
        //drive.followTrajectory(Line1);
        //drive.turn(Math.toRadians(-135));
        //drive.followTrajectory(Traj2);
        //drive.followTrajectory(Traj25);
        //drive.turn(Math.toRadians(180));
        //drive.followTrajectory(Traj3);
        //drive.followTrajectory(Traj4);


    }
}
