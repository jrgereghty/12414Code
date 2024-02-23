package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVDetectTeamProp;
import org.firstinspires.ftc.teamcode.drive.LevineAuto.BlueClosePoses;
import org.firstinspires.ftc.teamcode.drive.LevineAuto.BlueFarPoses;
import org.firstinspires.ftc.teamcode.drive.LevineAuto.RedClosePoses;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class JayMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode

    public OpMode opMode;
    public Pose2d startingPosition, startingPosition2, boardBack;
    public Pose2d testPosition;
    public Pose2d firstPlacementLeft, firstPlacementMid, firstPlacementRight;
    public Pose2d Turn2BoardLeft, Turn2BoardMid, Turn2BoardRight;
    public Pose2d PerpendicularBoardLeft, PerpendicularBoardMid, PerpendicularBoardRight;
    public Pose2d UnPerpendicularBoardLeft, UnPerpendicularBoardMid, UnPerpendicularBoardRight;

    public Pose2d DoorAlignmentBoard, TrussAlignmentBoard;
    public Pose2d doorStack, trussStack, emergencyStack;
    public Pose2d parkPrepare, parkFinish;
    public Pose2d reset4Left;//Legacy
    public Pose2d whitePickupBMid;

    //Define all hardware
    public VoltageSensor batteryVoltageSensor;
    public DcMotor frontLeft, frontRight, backLeft, backRight, hangLeft, hangRight, slide;
    public Servo clawHAngle, clawVAngle, slideLAngle, slideRAngle, clawL, clawR;
    public WebcamName bonoboCam;
    public BNO055IMU gyro;//Can we do it?

    public boolean halfSpeedToggle = true;
    public boolean aLast = false;

    public boolean drivingReverse = false;
    public boolean yLast = false;

    public boolean clawLOpen = false;
    public boolean leftBumper2Last = false;

    public boolean clawROpen = false;
    public boolean rightBumper2Last = false;

    public boolean intake = true;
    public boolean ps2Last = false;

    // slidePos is a fraction of the total possible slide extension.
    public double slideLength = 0.0;
    public double slidePos = 0.0;

    public double yMovement;
    public double xMovement;
    public double rotation;
    public double drivePower;
    public double slidePower;
    public static double pos = 0.8;
    public static double hPos = 0.5;
    public static double vPos = 1;
    public static double stackNum = 0;

    public JayMap(OpMode opMode) {
        this.opMode = opMode;
    }

    public JayMap(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public static double getSlideVelocity(int trigger, double slidePos, double triggerDepth) {
        double velocity = 0.0;
        if (trigger == 1) {
            velocity = triggerDepth * (0.6 * Math.cos(0.5 * Math.PI * slidePos) + 0.4);
        } else if (trigger == -1) {
            velocity = triggerDepth * (-0.6 * Math.sin(0.5 * Math.PI * slidePos) - 0.4);
        }
        return (velocity);
    }

    public static double getSlideAngle(double slideLength) {
        return (Math.toDegrees(Math.acos(22.5 / slideLength)) / 47.8 - 1.4);
    }

    public static double getSlideAngle(double slideLength, double stackHeight) {
        return (Math.toDegrees(Math.acos((22.5-0.5*stackHeight) / slideLength)) / 47.8 - 1.4);
    }

    public static double getClawVAngle1(double slideLength) {
        return (-Math.toDegrees(Math.asin(22.5 / slideLength)) / 428.57 + 0.4);
    }

    public static double getClawVAngle1(double slideLength, double stackHeight) {
        return (-Math.toDegrees(Math.asin((22.5-0.5*stackHeight) / slideLength)) / 428.57 + 0.4);
    }

    public static double getClawVAngle2(double slideAngle) {
        return (slideAngle / 4 + 0.05);
    }

    public void closeLeftClaw() {
        clawL.setPosition(0);
    }

    public void closeRightClaw() {
        clawR.setPosition(0);
    }

    public void openLeftClaw() {
        clawL.setPosition(0.5);
    }

    public void openRightClaw() {
        clawR.setPosition(0.5);
    }

    public void setIntakeMode() {
        intake = true;
    }

    public void setPlacingMode() {
        intake = false;
    }

    public void setToAngle(double angle2) {
        clawHAngle.setPosition(angle2);
    }
    public void initSlideToPos(){slide.setTargetPosition(0);slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public void clawExtensionManager(int slideLength, int stackHeight){double slidePos = (slide.getCurrentPosition() / 537.7 * 4 * Math.PI / 41.1) * 41.1 + 38.5;double slideAngle = getSlideAngle(slidePos, stackHeight);
        clawVAngle.setPosition(getClawVAngle1(slidePos, stackHeight));slideLAngle.setPosition(slideAngle);slideRAngle.setPosition(slideAngle);//Save 384.5
    slide.setTargetPosition(slideLength);slide.setPower(1);}
    public void perpendicularBoardPlacement(double slideAngle, int intendedSlideLength){clawVAngle.setPosition(getClawVAngle2(slideAngle));
        slideRAngle.setPosition(slideAngle);slideLAngle.setPosition(slideAngle);slide.setTargetPosition(intendedSlideLength);slide.setPower(1);}
    public void slideToTarget(int intendedSlideLength, double power){slide.setTargetPosition(intendedSlideLength);slide.setPower(power);}
    public void resetCLaw4Park(){closeLeftClaw();closeRightClaw();clawVAngle.setPosition(0.9);}
    public void setAllSlidePoses(int intendedSlideLength, double intendedSlideAngle, double slidePower){slideToTarget(intendedSlideLength, slidePower);slideLAngle.setPosition(intendedSlideAngle);slideRAngle.setPosition(intendedSlideAngle);}
    public void setSlideAngle( double intendedSlideAngle){slideLAngle.setPosition(intendedSlideAngle);slideRAngle.setPosition(intendedSlideAngle);}

    public void init() {
        frontLeft = this.opMode.hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = this.opMode.hardwareMap.dcMotor.get("frontRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = this.opMode.hardwareMap.dcMotor.get("backLeft");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        backRight = this.opMode.hardwareMap.dcMotor.get("backRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangLeft = this.opMode.hardwareMap.dcMotor.get("hangLeft");
        hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        hangRight = this.opMode.hardwareMap.dcMotor.get("hangRight");
        hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = this.opMode.hardwareMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLAngle = this.opMode.hardwareMap.servo.get("slideLAngle");
        slideLAngle.setDirection(Servo.Direction.REVERSE);
        slideLAngle.setPosition(0.8);

        slideRAngle = this.opMode.hardwareMap.servo.get("slideRAngle");
        slideRAngle.setPosition(0.8);

        clawHAngle = this.opMode.hardwareMap.servo.get("clawHAngle");
        clawHAngle.scaleRange(0.04, 1);
        clawHAngle.setPosition(hPos);

        clawVAngle = this.opMode.hardwareMap.servo.get("clawVAngle");
        clawVAngle.scaleRange(0, 0.7);
        clawVAngle.setPosition(vPos);

        clawL = this.opMode.hardwareMap.servo.get("clawL");
        clawL.scaleRange(0.21, 0.605);
        clawL.setPosition(0);

        clawR = this.opMode.hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
        clawR.scaleRange(0.20, 0.595);
        clawR.setPosition(0);



        Telemetry telemetry = new MultipleTelemetry(this.opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        VoltageSensor batteryVoltageSensor = this.opMode.hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        telemetry.update();
    }


    /*
    public void initForApril(){
        WebcamName bonoboCam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

     */


    //HAPPY TIMES_________________+++___++__+_+_+_++_+__+_+_+_++_+__++_+__+_+_++__++__+:
    public void initForAuton(String autonType) {
        initPoses(autonType);
        initSlideToPos();
        //flipUp();
        //closeGrabber();
        //setRotatorUp();
        //setCorrectorMid();
        //resetSlidePoses();
    }

    public void initPoses(String autonType) {
        switch (autonType) {
            case "B_Clo_Door_Mid":
                startingPosition = new Pose2d(BlueClosePoses.xPosStartingPos, BlueClosePoses.yPosStartingPos, BlueClosePoses.headingStartingPos);
                startingPosition2 = new Pose2d(BlueClosePoses.xPosStartingPos2, BlueClosePoses.yPosStartingPos2, BlueClosePoses.headingStartingPos);
                firstPlacementLeft = new Pose2d(BlueClosePoses.xPosLeftSpikeMark, BlueClosePoses.yPosLeftSpikeMark, BlueClosePoses.headingLeftSpikeMark);
                firstPlacementMid = new Pose2d(BlueClosePoses.xPosMiddleSpikeMark, BlueClosePoses.yPosMiddleSpikeMark, BlueClosePoses.headingMiddleSpikeMark);
                firstPlacementRight = new Pose2d(BlueClosePoses.xPosRightSpikeMark, BlueClosePoses.yPosRightSpikeMark, BlueClosePoses.headingRightSpikeMark);

                Turn2BoardMid = new Pose2d(BlueClosePoses.xPosTurn2BoardMid, BlueClosePoses.yPosTurn2BoardMid, BlueClosePoses.headingTurn2BoardMid);

                boardBack = new Pose2d(BlueClosePoses.xPosBoardBack, BlueClosePoses.yPosBoardBack, BlueClosePoses.faceBoard);

                PerpendicularBoardLeft = new Pose2d(BlueClosePoses.xPosLeftBoardPlace, BlueClosePoses.yPosLeftBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardMid = new Pose2d(BlueClosePoses.xPosMidBoardPlace, BlueClosePoses.yPosMidBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardRight = new Pose2d(BlueClosePoses.xPosRightBoardPlace, BlueClosePoses.yPosRightBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);

                DoorAlignmentBoard = new Pose2d(BlueClosePoses.xPosDoorLaneAlignmentBoard, BlueClosePoses.yPosDoorLaneAlignmentBoard, BlueClosePoses.headingDoorLaneAlignmentBoard);
                doorStack = new Pose2d(BlueClosePoses.xPosWhitePickupMid, BlueClosePoses.yPosWhitePickupMid, BlueClosePoses.headingWhitePickupMid);
                parkPrepare = new Pose2d(BlueClosePoses.xPosStartParkMid, BlueClosePoses.yPosStartParkMid, BlueClosePoses.faceBoard);
                parkFinish = new Pose2d(BlueClosePoses.xPosEndParkMid, BlueClosePoses.yPosEndParkMid, BlueClosePoses.faceBoard);
                break;
            case "B_Clo_Door_Edge":
                startingPosition = new Pose2d(BlueClosePoses.xPosStartingPos, BlueClosePoses.yPosStartingPos, BlueClosePoses.headingStartingPos);
                startingPosition2 = new Pose2d(BlueClosePoses.xPosStartingPos2, BlueClosePoses.yPosStartingPos2, BlueClosePoses.headingStartingPos);
                firstPlacementLeft = new Pose2d(BlueClosePoses.xPosLeftSpikeMark, BlueClosePoses.yPosLeftSpikeMark, BlueClosePoses.headingLeftSpikeMark);
                firstPlacementMid = new Pose2d(BlueClosePoses.xPosMiddleSpikeMark, BlueClosePoses.yPosMiddleSpikeMark, BlueClosePoses.headingMiddleSpikeMark);
                firstPlacementRight = new Pose2d(BlueClosePoses.xPosRightSpikeMark, BlueClosePoses.yPosRightSpikeMark, BlueClosePoses.headingRightSpikeMark);

                Turn2BoardMid = new Pose2d(BlueClosePoses.xPosTurn2BoardMid, BlueClosePoses.yPosTurn2BoardMid, BlueClosePoses.headingTurn2BoardMid);

                boardBack = new Pose2d(BlueClosePoses.xPosBoardBack, BlueClosePoses.yPosBoardBack, BlueClosePoses.faceBoard);

                PerpendicularBoardLeft = new Pose2d(BlueClosePoses.xPosLeftBoardPlace, BlueClosePoses.yPosLeftBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardMid = new Pose2d(BlueClosePoses.xPosMidBoardPlace, BlueClosePoses.yPosMidBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardRight = new Pose2d(BlueClosePoses.xPosRightBoardPlace, BlueClosePoses.yPosRightBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);

                DoorAlignmentBoard = new Pose2d(BlueClosePoses.xPosDoorLaneAlignmentBoard, BlueClosePoses.yPosDoorLaneAlignmentBoard, BlueClosePoses.headingDoorLaneAlignmentBoard);
                doorStack = new Pose2d(BlueClosePoses.xPosWhitePickupMid, BlueClosePoses.yPosWhitePickupMid, BlueClosePoses.headingWhitePickupMid);
                parkPrepare = new Pose2d(BlueClosePoses.xPosStartParkEdge, BlueClosePoses.yPosStartParkEdge, BlueClosePoses.faceBoard);
                parkFinish = new Pose2d(BlueClosePoses.xPosEndParkEdge, BlueClosePoses.yPosEndParkEdge, BlueClosePoses.faceBoard);
                break;
            case "B_Clo_Door_Mid_L":
                startingPosition = new Pose2d(BlueClosePoses.xPosStartingPos, BlueClosePoses.yPosStartingPos, BlueClosePoses.headingStartingPos);
                firstPlacementLeft = new Pose2d(BlueClosePoses.xPosLine4StartLeft, BlueClosePoses.yPosLine4StartLeft, BlueClosePoses.headingLine4StartLeft);
                firstPlacementMid = new Pose2d(BlueClosePoses.xPosLine4StartMid, BlueClosePoses.yPosLine4StartMid, BlueClosePoses.headingLine4StartMid);
                firstPlacementRight = new Pose2d(BlueClosePoses.xPosLine4StartRight, BlueClosePoses.yPosLine4StartRight, BlueClosePoses.headingLine4StartRight);
                reset4Left = new Pose2d(BlueClosePoses.xPosReset4Left, BlueClosePoses.yPosReset4Left, BlueClosePoses.faceBoard);

                PerpendicularBoardLeft = new Pose2d(BlueClosePoses.xPosLeftBoardPlace, BlueClosePoses.yPosLeftBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardMid = new Pose2d(BlueClosePoses.xPosMidBoardPlace, BlueClosePoses.yPosMidBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardRight = new Pose2d(BlueClosePoses.xPosRightBoardPlace, BlueClosePoses.yPosRightBoardPlace, BlueClosePoses.PerpendicularBoardPlacementHeading);
                break;
            case "R_Clo_Door_Mid":
                startingPosition = new Pose2d(RedClosePoses.xPosStartingPos, RedClosePoses.yPosStartingPos, RedClosePoses.headingStartingPos);
                startingPosition2 = new Pose2d(RedClosePoses.xPosStartingPos2, RedClosePoses.yPosStartingPos2, RedClosePoses.headingStartingPos);
                firstPlacementLeft = new Pose2d(RedClosePoses.xPosLeftSpikeMark, RedClosePoses.yPosLeftSpikeMark, RedClosePoses.headingLeftSpikeMark);
                firstPlacementMid = new Pose2d(RedClosePoses.xPosMiddleSpikeMark, RedClosePoses.yPosMiddleSpikeMark, RedClosePoses.headingMiddleSpikeMark);
                firstPlacementRight = new Pose2d(RedClosePoses.xPosRightSpikeMark, RedClosePoses.yPosRightSpikeMark, RedClosePoses.headingRightSpikeMark);

                Turn2BoardMid = new Pose2d(RedClosePoses.xPosTurn2BoardMid, RedClosePoses.yPosTurn2BoardMid, RedClosePoses.headingTurn2BoardMid);

                boardBack = new Pose2d(RedClosePoses.xPosBoardBack, RedClosePoses.yPosBoardBack, RedClosePoses.faceBoard);

                PerpendicularBoardLeft = new Pose2d(RedClosePoses.xPosLeftBoardPlace, RedClosePoses.yPosLeftBoardPlace, RedClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardMid = new Pose2d(RedClosePoses.xPosMidBoardPlace, RedClosePoses.yPosMidBoardPlace, RedClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardRight = new Pose2d(RedClosePoses.xPosRightBoardPlace, RedClosePoses.yPosRightBoardPlace, RedClosePoses.PerpendicularBoardPlacementHeading);

                DoorAlignmentBoard = new Pose2d(RedClosePoses.xPosDoorLaneAlignmentBoard, RedClosePoses.yPosDoorLaneAlignmentBoard, RedClosePoses.headingDoorLaneAlignmentBoard);
                doorStack = new Pose2d(RedClosePoses.xPosWhitePickupMid, RedClosePoses.yPosWhitePickupMid, RedClosePoses.headingWhitePickupMid);
                parkPrepare = new Pose2d(RedClosePoses.xPosStartParkMid, RedClosePoses.yPosStartParkMid, RedClosePoses.faceBoard);
                parkFinish = new Pose2d(RedClosePoses.xPosEndParkMid, RedClosePoses.yPosEndParkMid, RedClosePoses.faceBoard);



                //ParkP
                break;
            case "R_Clo_Door_Edge":
                startingPosition = new Pose2d(RedClosePoses.xPosStartingPos, RedClosePoses.yPosStartingPos, RedClosePoses.headingStartingPos);
                startingPosition2 = new Pose2d(RedClosePoses.xPosStartingPos2, RedClosePoses.yPosStartingPos2, RedClosePoses.headingStartingPos);
                firstPlacementLeft = new Pose2d(RedClosePoses.xPosLeftSpikeMark, RedClosePoses.yPosLeftSpikeMark, RedClosePoses.headingLeftSpikeMark);
                firstPlacementMid = new Pose2d(RedClosePoses.xPosMiddleSpikeMark, RedClosePoses.yPosMiddleSpikeMark, RedClosePoses.headingMiddleSpikeMark);
                firstPlacementRight = new Pose2d(RedClosePoses.xPosRightSpikeMark, RedClosePoses.yPosRightSpikeMark, RedClosePoses.headingRightSpikeMark);

                Turn2BoardMid = new Pose2d(RedClosePoses.xPosTurn2BoardMid, RedClosePoses.yPosTurn2BoardMid, RedClosePoses.headingTurn2BoardMid);

                boardBack = new Pose2d(RedClosePoses.xPosBoardBack, RedClosePoses.yPosBoardBack, RedClosePoses.faceBoard);

                PerpendicularBoardLeft = new Pose2d(RedClosePoses.xPosLeftBoardPlace, RedClosePoses.yPosLeftBoardPlace, RedClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardMid = new Pose2d(RedClosePoses.xPosMidBoardPlace, RedClosePoses.yPosMidBoardPlace, RedClosePoses.PerpendicularBoardPlacementHeading);
                PerpendicularBoardRight = new Pose2d(RedClosePoses.xPosRightBoardPlace, RedClosePoses.yPosRightBoardPlace, RedClosePoses.PerpendicularBoardPlacementHeading);

                DoorAlignmentBoard = new Pose2d(RedClosePoses.xPosDoorLaneAlignmentBoard, RedClosePoses.yPosDoorLaneAlignmentBoard, RedClosePoses.headingDoorLaneAlignmentBoard);
                doorStack = new Pose2d(RedClosePoses.xPosWhitePickupMid, RedClosePoses.yPosWhitePickupMid, RedClosePoses.headingWhitePickupMid);
                parkPrepare = new Pose2d(RedClosePoses.xPosStartParkEdge, RedClosePoses.yPosStartParkEdge, RedClosePoses.faceBoard);
                parkFinish = new Pose2d(RedClosePoses.xPosEndParkEdge, RedClosePoses.yPosEndParkEdge, RedClosePoses.faceBoard);



                //ParkP
                break;
            case "B_Far_Door_Mid":
                startingPosition = new Pose2d(BlueFarPoses.xPosStartingPos, BlueFarPoses.yPosStartingPos, BlueFarPoses.headingStartingPos);
                startingPosition2 = new Pose2d(BlueFarPoses.xPosStartingPos2, BlueFarPoses.yPosStartingPos2, BlueFarPoses.headingStartingPos);
                firstPlacementLeft = new Pose2d(BlueFarPoses.xPosLeftSpikeMark, BlueFarPoses.yPosLeftSpikeMark, BlueFarPoses.headingLeftSpikeMark);
                firstPlacementMid = new Pose2d(BlueFarPoses.xPosMiddleSpikeMark, BlueFarPoses.yPosMiddleSpikeMark, BlueFarPoses.headingMiddleSpikeMark);
                firstPlacementRight = new Pose2d(BlueFarPoses.xPosRightSpikeMark, BlueFarPoses.yPosRightSpikeMark, BlueFarPoses.headingRightSpikeMark);

                whitePickupBMid = new Pose2d(BlueFarPoses.xPosWhitePickupMid, BlueFarPoses.yPosWhitePickupMid, BlueFarPoses.headingWhitePickup);
                DoorAlignmentBoard = new Pose2d(BlueFarPoses.xPosDoorLaneAlignmentBoard, BlueFarPoses.yPosDoorLaneAlignmentBoard, BlueFarPoses.headingDoorLaneAlignmentBoard);


                PerpendicularBoardLeft = new Pose2d(BlueFarPoses.xPosLeftBoardPlace, BlueFarPoses.yPosLeftBoardPlace, 0);
                PerpendicularBoardMid = new Pose2d(BlueFarPoses.xPosMidBoardPlace, BlueFarPoses.yPosMidBoardPlace, 0);
                PerpendicularBoardRight = new Pose2d(BlueFarPoses.xPosRightBoardPlace, BlueFarPoses.yPosRightBoardPlace,0);

                parkPrepare = new Pose2d(BlueFarPoses.xPosStartParkMid, BlueFarPoses.yPosStartParkMid, RedClosePoses.faceBoard);
                parkFinish = new Pose2d(BlueFarPoses.xPosEndParkEdge, BlueFarPoses.yPosEndParkMid, RedClosePoses.faceBoard);
                break;
        }
    }
}
