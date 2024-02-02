package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LevineLocalization.PointFollower;
import org.firstinspires.ftc.teamcode.LevineLocalization.PosesAndActions;
import org.firstinspires.ftc.teamcode.CenterStageAuton.OpenCVDetectTeamProp;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class JayMap {
    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();

    //Define opMode

    public OpMode opMode;

    //Define all hardware
    public VoltageSensor batteryVoltageSensor;
    public DcMotor frontLeft, frontRight, backLeft, backRight, hangLeft, hangRight, slide;
    public Servo clawHAngle, clawVAngle, slideLAngle, slideRAngle, clawL, clawR;

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

    public JayMap(OpMode opMode){
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
        return(velocity);
    }

    public static double getSlideAngle(double slideLength) {
        return(Math.toDegrees(Math.acos(22.5 / slideLength)) / 47.8 - 1.4);
    }

    public static double getClawVAngle1(double slideLength) {
        return(-Math.toDegrees(Math.asin(22.5 / slideLength)) / 428.57 + 0.4);
    }

    public static double getClawVAngle2(double slideAngle) {
        return(slideAngle / 3.43 + 0.117);
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
    public void setIntakeMode() {intake = true;}
    public void setPlacingMode() {intake = false;}

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
}
