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
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.drive.JayMap;
import org.firstinspires.ftc.teamcode.LevineLocalization.ActionRunnerCenterStageAuton;
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

    JayMap jayBot = new JayMap(this);
    ActionRunnerCenterStageAuton actionRunner = new ActionRunnerCenterStageAuton(this, jayBot);
    PointFollower follower = new PointFollower(this, actionRunner);

    @Override
    public void runOpMode() {
        jayBot.init();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

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
