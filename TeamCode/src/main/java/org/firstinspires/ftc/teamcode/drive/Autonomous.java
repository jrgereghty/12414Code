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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "blue")

public class Autonomous extends LinearOpMode {
    public static VisionPortal visionPortal;

    public static WebcamName cam1;
    public static AprilTagProcessor aprilTag;

    VisionPortal myVisionPortal;


    @Override
    public void runOpMode() {
        cam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        CameraInit cameraPro = new CameraInit(hardwareMap);
        //myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"));

        cameraPro.switchToFirstPipeline();
        telemetry.addLine("Status: Initialized");

        DcMotor slideLeft;
        DcMotor slideRight;
        Servo slideAngleLeft;
        Servo slideAngleRight;
        Servo boxLid;

        DcMotor slurp;






        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory Trajegy = drive.trajectoryBuilder(startPose)
                .forward(20)

                .build();

        Trajectory Traj1 = drive.trajectoryBuilder(startPose)

                .splineTo(new Vector2d(-36.16, -35.18), Math.toRadians(89.66))
                .splineToConstantHeading(new Vector2d(-31.42, -59.56), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(8.02, -60.38), Math.toRadians(0.00))
                .build();
        Trajectory Line1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(42.22, -36.33, Math.toRadians(0.00)))

                .build();
        Trajectory Traj2 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-12.11, -60.38), Math.toRadians(180.00))
                .build();
        Trajectory Traj25 = drive.trajectoryBuilder(startPose)

                .splineToLinearHeading(new Pose2d(-56.13, -44.18, Math.toRadians(150.00)), Math.toRadians(150.00))
                .build();

        Trajectory Traj3 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-28.47, -61.53, Math.toRadians(0.29)), Math.toRadians(0.29))

                .build();
        Trajectory Traj4 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(18.98, -56.13), Math.toRadians(19.34))
                .splineTo(new Vector2d(47.78, -38.13), Math.toRadians(4.04))
                .build();

        waitForStart(); //___________________________________________________________________

        if(isStopRequested()) return;

        slideAngleLeft = hardwareMap.servo.get("slideAngleLeft");
        slideAngleRight = hardwareMap.servo.get("slideAngleRight");
        slideAngleLeft.scaleRange(0.425, 0.995);
        slideAngleRight.scaleRange(0, 0.57);
        slideAngleLeft.setPosition(0);
        slideAngleRight.setPosition(1);

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
        telemetry.addLine(cameraPro.getPipeline1Output());
        telemetry.update();


        slideAngleLeft.setPosition(slideAngleLeft.getPosition() + 0.1);
        slideAngleRight.setPosition(slideAngleRight.getPosition() - 0.1);

        







        //drive.followTrajectory(Traj1);
        drive.turn(Math.toRadians(90));

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





        drive.followTrajectory(Trajegy);
        //drive.followTrajectory(Line1);
        //drive.turn(Math.toRadians(-135));
        //drive.followTrajectory(Traj2);
        //drive.followTrajectory(Traj25);
        //drive.turn(Math.toRadians(180));
        //drive.followTrajectory(Traj3);
        //drive.followTrajectory(Traj4);

        
    }
}