package org.firstinspires.ftc.teamcode;

//luh TeleOp
//default position pos = 0.9, vpos = 1

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "purple")
@Config

public class TeleOp extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor slide;

    DcMotor hangLeft;
    DcMotor hangRight;

    Servo clawHAngle;
    Servo clawVAngle;
    Servo slideLAngle;
    Servo slideRAngle;
    Servo clawL;
    Servo clawR;

    MecanumDrive drive;

    boolean halfSpeedToggle = false;
    boolean rightBumperLast = false;

    boolean drivingReverse = false;
    boolean leftBumperLast = false;

    // slidePos is a fraction of the total possible slide extension.
    double slidePos = 0.0;

    double yMovement;
    double xMovement;
    double rotation;
    double drivePower;
    double slidePower;
    public static double pos = 0.5;
    public static double hPos = 0.5;
    public static double vPos = 0.5;
    public static double lPos = 0.5;
    public static double rPos = 0.5;

    private static double getSlideVelocity(int trigger, double slidePos, double triggerDepth) {
        double velocity = 0.0;
        if (trigger == 1) {
            velocity = 0.4 * triggerDepth * Math.cos(0.5 * Math.PI * slidePos) + 0.3;
        } else if (trigger == -1) {
            velocity = -0.4 * triggerDepth * Math.sin(0.5 * Math.PI * slidePos) - 0.3;
        }
        return(velocity);
    }

    public void init() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        backRight = hardwareMap.dcMotor.get("backRight");
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangLeft = hardwareMap.dcMotor.get("hangLeft");
        hangLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangRight = hardwareMap.dcMotor.get("hangRight");
        hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangRight.setDirection(DcMotorSimple.Direction.REVERSE);

        slide = hardwareMap.dcMotor.get("slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideLAngle = hardwareMap.servo.get("slideLAngle");
        slideLAngle.setDirection(Servo.Direction.REVERSE);
        slideLAngle.setPosition(0.5);

        slideRAngle = hardwareMap.servo.get("slideRAngle");
        slideRAngle.setPosition(0.5);

        clawHAngle = hardwareMap.servo.get("clawHAngle");
        clawHAngle.scaleRange(0.04, 1);
        clawHAngle.setPosition(hPos);

        clawVAngle = hardwareMap.servo.get("clawVAngle");
        clawVAngle.scaleRange(0, 0.6);
        clawVAngle.setPosition(vPos);

        clawL = hardwareMap.servo.get("clawL");
        clawL.scaleRange(0.21, 0.605);
        clawL.setPosition(lPos);

        clawR = hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
        clawR.scaleRange(0.20, 0.595);
        clawR.setPosition(rPos);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

    }

    public void loop() {

        xMovement = gamepad1.left_stick_x;
        yMovement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        drivePower = Math.max(Math.max(Math.abs(yMovement), Math.abs(xMovement)), Math.abs(rotation));

        if (!rightBumperLast && gamepad1.right_bumper) {
            halfSpeedToggle = !halfSpeedToggle;
        }
        if (halfSpeedToggle) {
            drivePower *= 0.5;
        }
        rightBumperLast = gamepad1.right_bumper;

        slidePos = slide.getCurrentPosition() / 537.7 * 4 * Math.PI / 54.864;
        if (gamepad1.right_trigger > 0.01) {
            slidePower = getSlideVelocity(1, slidePos, Math.pow(gamepad1.right_trigger, 2));
            slide.setPower(slidePower);
        } else if (gamepad1.left_trigger > 0.01) {
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(gamepad1.left_trigger, 2));
            slide.setPower(slidePower);
        } else {
            slide.setPower(0);
        }

        hangLeft.setPower(0);
        hangRight.setPower(0);
        if (gamepad1.y) {
            hangLeft.setPower(-1);
            hangRight.setPower(-1);
        }
        if (gamepad1.a) {
            hangLeft.setPower(1);
            hangRight.setPower(1);
        }

        if (!leftBumperLast && gamepad1.left_bumper) {
            drivingReverse = !drivingReverse;
        }
        if (drivingReverse) {
            xMovement *= -1;
            yMovement *= -1;
        }
        leftBumperLast = gamepad1.left_bumper;

        /*
        if (gamepad1.dpad_up) {
            clawVAngle.setPosition(clawVAngle.getPosition() + 0.001);
        }
        if (gamepad1.dpad_down) {
            clawVAngle.setPosition(clawVAngle.getPosition() - 0.001);
        }
        if (gamepad1.dpad_right) {
            clawHAngle.setPosition(clawHAngle.getPosition() - 0.001);
        }
        if (gamepad1.dpad_left) {
            clawHAngle.setPosition(clawHAngle.getPosition() + 0.001);
        }
        */

        clawL.setPosition(lPos);
        clawR.setPosition(rPos);
        clawHAngle.setPosition(hPos);
        clawVAngle.setPosition(vPos);
        slideLAngle.setPosition(pos);
        slideRAngle.setPosition(pos);

        telemetry.addData("slidePos", slidePos);
        telemetry.addData("halfSpeed", halfSpeedToggle);
        telemetry.addData("drivingReverse", drivingReverse);
        telemetry.addData("clawHAngle", clawHAngle.getPosition());
        telemetry.addData("clawVAngle", clawVAngle.getPosition());
        updateTelemetry(telemetry);

        drive.moveInTeleop(xMovement, yMovement, rotation, drivePower);

    }
}