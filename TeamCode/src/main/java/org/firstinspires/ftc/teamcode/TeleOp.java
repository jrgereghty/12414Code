package org.firstinspires.ftc.teamcode;

//luh TeleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "purple")

public class TeleOp extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor slideLeft;
    DcMotor slideRight;
    Servo slideAngleLeft;
    Servo slideAngleRight;
    Servo boxLid;
    Servo boxAngle;

    DcMotor slurp;

    MecanumDrive drive;

    boolean halfSpeedToggle = false;
    boolean rightBumperLast = false;
    boolean boxLidOpenToggle = false;
    boolean dpadDownLast = false;
    boolean boxUpToggle = true;
    boolean aLast = false;
    boolean slidesUpToggle = true;
    boolean dpadUpLast = false;
    boolean slurpToggle = false;
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

        slideAngleLeft = hardwareMap.servo.get("slideAngleLeft");
        slideAngleRight = hardwareMap.servo.get("slideAngleRight");
        slideAngleLeft.scaleRange(0.3, 0.43);
        slideAngleRight.scaleRange(0.57, 0.7);
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
        slideRight.setDirection(DcMotor.Direction.REVERSE);

        boxLid = hardwareMap.servo.get("boxLid");
        boxLid.scaleRange(0, 0.7);
        boxLid.setPosition(1);

        boxAngle = hardwareMap.servo.get("boxAngle");
        boxAngle.scaleRange(0.4, 1);
        boxAngle.setPosition(1);

        slurp = hardwareMap.dcMotor.get("slurp");
        slurp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slurp.setDirection(DcMotor.Direction.REVERSE);

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

        slideLeftPos = slideLeft.getCurrentPosition() / 537.7 * 4 * Math.PI / 97;
        slideRightPos = slideRight.getCurrentPosition() / 537.7 * 4 * Math.PI / 97;
        if (gamepad1.right_trigger > 0.1) {
            slideLeftPower = getSlideVelocity(1, slideLeftPos, gamepad1.right_trigger);
            slideRightPower = getSlideVelocity(1, slideRightPos, gamepad1.right_trigger);
            slideLeft.setPower(slideLeftPower);
            slideRight.setPower(slideRightPower);
        } else if (gamepad1.left_trigger > 0.1) {
            slideLeftPower = getSlideVelocity(-1, slideLeftPos, gamepad1.left_trigger);
            slideRightPower = getSlideVelocity(-1, slideRightPos, gamepad1.left_trigger);
            slideLeft.setPower(slideLeftPower);
            slideRight.setPower(slideRightPower);
        } else {
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }

        if (!dpadUpLast && gamepad1.dpad_up) {
            slidesUpToggle = !slidesUpToggle;
        }
        if (slidesUpToggle) {
            slideAngleLeft.setPosition(0);
            slideAngleRight.setPosition(1);
        } else {
            slideAngleLeft.setPosition(1);
            slideAngleRight.setPosition(0);
        }
        dpadUpLast = gamepad1.dpad_up;

        if (!dpadDownLast && gamepad1.dpad_down) {
            boxLidOpenToggle = !boxLidOpenToggle;
        }
        if (boxLidOpenToggle) {
            if (slidesUpToggle) {
                boxLid.setPosition(0);
            } else {
                boxLid.setPosition(0.5);
            }
        } else {
            boxLid.setPosition(1);
        }
        dpadDownLast = gamepad1.dpad_down;

        if (!aLast && gamepad1.a) {
            boxUpToggle = !boxUpToggle;
        }
        if (boxUpToggle) {
            boxAngle.setPosition(1);
        } else {
            boxAngle.setPosition(0);
        }
        aLast = gamepad1.a;

        if (!leftBumperLast && gamepad1.left_bumper) {
            slurpToggle = !slurpToggle;
        }
        if (slurpToggle) {
            slurp.setPower(0.8);
        } else {
            slurp.setPower(0);
        }
        leftBumperLast = gamepad1.left_bumper;

        telemetry.addData("slideLeftPos", slideLeftPos);
        telemetry.addData("slideRightPos", slideRightPos);
        telemetry.addData("halfSpeed", halfSpeedToggle);
        telemetry.addData("slideAngleLeft", slideAngleLeft.getPosition());
        telemetry.addData("slideAngleRight", slideAngleRight.getPosition());
        telemetry.addData("slideLeft", slideLeftPower);
        telemetry.addData("slideRight", slideRightPower);
        telemetry.addData("boxAngle", boxAngle.getPosition());
        telemetry.addData("boxLid", boxLid.getPosition());
        telemetry.addData("slidesUpToggle", slidesUpToggle);
        updateTelemetry(telemetry);

        drive.moveInTeleop(xMovement, yMovement, rotation, drivePower);

    }
}