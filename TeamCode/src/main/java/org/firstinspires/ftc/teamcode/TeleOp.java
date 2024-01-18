package org.firstinspires.ftc.teamcode;

//luh TeleOp

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

    DcMotor slideLeft;
    DcMotor slideRight;

    DcMotor hangLeft;
    DcMotor hangRight;

    Servo clawHAngle;
    Servo clawVAngle;
    Servo slideLAngle;
    Servo slideRAngle;

    MecanumDrive drive;

    boolean halfSpeedToggle = false;
    boolean rightBumperLast = false;

    boolean slidesUpToggle = false;

    boolean drivingReverse = false;
    boolean leftBumperLast = false;

    // slidePos is a fraction of the total possible slide extension.
    double slideLeftPos = 0.0;
    double slideRightPos = 0.0;

    double yMovement;
    double xMovement;
    double rotation;
    double drivePower;
    double slideLeftPower;
    double slideRightPower;
    public static double pos;

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

        /*slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setDirection(DcMotor.Direction.REVERSE);*/

        slideLAngle = hardwareMap.servo.get("slideLAngle");
        slideLAngle.setDirection(Servo.Direction.REVERSE);
        slideLAngle.setPosition(0.5);

        slideRAngle = hardwareMap.servo.get("slideRAngle");
        slideRAngle.setPosition(0.5);

        clawHAngle = hardwareMap.servo.get("clawHAngle");
        clawHAngle.scaleRange(0.036, 1);
        clawHAngle.setPosition(0.5);

        clawVAngle = hardwareMap.servo.get("clawVAngle");
        clawVAngle.scaleRange(0.27, 1);
        clawVAngle.setPosition(0.195);


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

        /*slideLeftPos = slideLeft.getCurrentPosition() / 537.7 * 4 * Math.PI / 97;
        slideRightPos = slideRight.getCurrentPosition() / 537.7 * 4 * Math.PI / 97;
        if (gamepad1.right_trigger > 0.01) {
            slideLeftPower = getSlideVelocity(1, slideLeftPos, Math.pow(gamepad1.right_trigger, 2));
            slideRightPower = getSlideVelocity(1, slideRightPos, Math.pow(gamepad1.right_trigger, 2));
            slideLeft.setPower(slideLeftPower);
            slideRight.setPower(slideRightPower);
        } else if (gamepad1.left_trigger > 0.01) {
            slideLeftPower = getSlideVelocity(-1, slideLeftPos, Math.pow(gamepad1.left_trigger, 2));
            slideRightPower = getSlideVelocity(-1, slideRightPos, Math.pow(gamepad1.left_trigger, 2));
            slideLeft.setPower(slideLeftPower);
            slideRight.setPower(slideRightPower);
        } else {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }*/

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

        slideLAngle.setPosition(pos);
        slideRAngle.setPosition(pos);

        telemetry.addData("slideLeftPos", slideLeftPos);
        telemetry.addData("slideRightPos", slideRightPos);
        telemetry.addData("halfSpeed", halfSpeedToggle);
        telemetry.addData("drivingReverse", drivingReverse);
        telemetry.addData("clawHAngle", clawHAngle.getPosition());
        telemetry.addData("clawVAngle", clawVAngle.getPosition());
        updateTelemetry(telemetry);

        drive.moveInTeleop(xMovement, yMovement, rotation, drivePower);

    }
}