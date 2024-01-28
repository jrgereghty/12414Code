package org.firstinspires.ftc.teamcode;

//luh TeleOp
//default position pos = 0.9, vpos = 1

/*flat positions:
pos = 0 (70 deg), vpos = 0.205, slidePos = 0.299
pos = 0.05 (72 deg), vpos = 0.21, slidePos = 0.421
pos = 0.1 (75 deg), vpos = 0.214, slidePos = 0.573
*/

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

    boolean halfSpeedToggle = true;
    boolean aLast = false;

    boolean drivingReverse = false;
    boolean yLast = false;

    boolean clawLOpen = false;
    boolean leftBumper2Last = false;

    boolean clawROpen = false;
    boolean rightBumper2Last = false;

    boolean intake = true;
    boolean ps2Last = false;

    // slidePos is a fraction of the total possible slide extension.
    double slideLength = 0.0;
    double slidePos = 0.0;

    double yMovement;
    double xMovement;
    double rotation;
    double drivePower;
    double slidePower;
    public static double pos = 0.5;
    public static double hPos = 0.5;
    public static double vPos = 0.5;

    private static double getSlideVelocity(int trigger, double slidePos, double triggerDepth) {
        double velocity = 0.0;
        if (trigger == 1) {
            velocity = triggerDepth * (0.6 * Math.cos(0.5 * Math.PI * slidePos) + 0.4);
        } else if (trigger == -1) {
            velocity = triggerDepth * (-0.6 * Math.sin(0.5 * Math.PI * slidePos) - 0.4);
        }
        return(velocity);
    }

    private static double getSlideAngle(double slideLength) {
        if (Math.toDegrees(Math.acos(22.5 / slideLength)) / 47.8 - 1.4 >= 0) {
            return(Math.toDegrees(Math.acos(22.5 / slideLength)) / 47.8 - 1.4);
        } else {
            return(0);
        }
    }

    private static double getClawVAngle1(double slideLength) {
        return(-Math.toDegrees(Math.asin(22.5 / slideLength)) / 500 + 0.47);
    }

    private static double getClawVAngle2(double slideAngle) {
        return(slideAngle / 4 + 0.1);
    }

    @Override
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
        hangLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        hangRight = hardwareMap.dcMotor.get("hangRight");
        hangRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = hardwareMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLAngle = hardwareMap.servo.get("slideLAngle");
        slideLAngle.setDirection(Servo.Direction.REVERSE);
        slideLAngle.setPosition(0.8);

        slideRAngle = hardwareMap.servo.get("slideRAngle");
        slideRAngle.setPosition(0.8);

        clawHAngle = hardwareMap.servo.get("clawHAngle");
        clawHAngle.scaleRange(0.04, 1);
        clawHAngle.setPosition(hPos);

        clawVAngle = hardwareMap.servo.get("clawVAngle");
        clawVAngle.setPosition(0.7);
        clawVAngle.scaleRange(0, 0.6);

        clawL = hardwareMap.servo.get("clawL");
        clawL.scaleRange(0.21, 0.605);
        clawL.setPosition(0.5);

        clawR = hardwareMap.servo.get("clawR");
        clawR.setDirection(Servo.Direction.REVERSE);
        clawR.scaleRange(0.20, 0.595);
        clawR.setPosition(0.5);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

    }

    @Override
    public void loop() {

        xMovement = gamepad1.left_stick_x;
        yMovement = gamepad1.left_stick_y;
        rotation = gamepad1.right_stick_x;
        drivePower = Math.max(Math.max(Math.abs(yMovement), Math.abs(xMovement)), Math.abs(rotation));

        if (!aLast && gamepad1.a) {
            halfSpeedToggle = !halfSpeedToggle;
        }
        if (halfSpeedToggle) {
            drivePower *= 0.5;
        }
        aLast = gamepad1.a;

        if (!yLast && gamepad1.y) {
            drivingReverse = !drivingReverse;
        }
        if (drivingReverse) {
            xMovement *= -1;
            yMovement *= -1;
        }
        yLast = gamepad1.y;

        /*
        if (Math.abs(gamepad1.right_stick_y) > 0.01) {
            pos += Math.pow(gamepad1.right_stick_y, 3) * 0.05;
        }
        slideLAngle.setPosition(pos);
        slideRAngle.setPosition(pos);
        */

        if (!rightBumper2Last && gamepad2.right_bumper) {
            clawROpen = !clawROpen;
        }
        if (clawROpen) {
            clawR.setPosition(0.5);
        } else {
            clawR.setPosition(0);
        }
        rightBumper2Last = gamepad2.right_bumper;

        if (!leftBumper2Last && gamepad2.left_bumper) {
            clawLOpen = !clawLOpen;
        }
        if (clawLOpen) {
            clawL.setPosition(0.5);
        } else {
            clawL.setPosition(0);
        }
        leftBumper2Last = gamepad2.left_bumper;

        slidePos = slide.getCurrentPosition() / 537.7 * 4 * Math.PI / 41.1;
        slideLength = slidePos * 41.1 + 38.5;
        if (gamepad2.right_trigger > 0.01) {
            slidePower = getSlideVelocity(1, slidePos, Math.pow(gamepad2.right_trigger, 3));
            slide.setPower(slidePower);
        } else if (gamepad2.left_trigger > 0.01) {
            slidePower = getSlideVelocity(-1, slidePos, Math.pow(gamepad2.left_trigger, 3));
            slide.setPower(slidePower);
        } else {
            if (slidePos <= 0.08) {
                slide.setPower(-0.5);
            } else if (slidePos >= 0.92) {
                slide.setPower(0.5);
            }
            slide.setPower(0);
        }

        if (!ps2Last && gamepad2.ps) {
            intake = !intake;
            hPos = 0.5;
            if (!intake) {
                pos = 0.5;
                vPos = getClawVAngle2(pos);
            }
        }
        if (intake) {
            pos = getSlideAngle(slideLength);
            vPos = getClawVAngle1(slideLength);
        } else {
            if (gamepad2.left_stick_y >= 0.005 && pos <= 0.99) {
                pos += gamepad2.left_stick_y * 0.01;
            }
            if (gamepad2.left_stick_y <= -0.005 && pos >= 0.01) {
                pos += gamepad2.left_stick_y * 0.01;
            }
            vPos = getClawVAngle2(pos);
        }
        if (gamepad2.right_stick_x >= 0.005 && hPos >= 0.01) {
            hPos += -gamepad2.right_stick_x * 0.01;
        }
        if (gamepad2.right_stick_x <= -0.005 && hPos <= 0.99) {
            hPos += -gamepad2.right_stick_x * 0.01;
        }
        if (hPos > 1) {
            hPos = 1;
        } else if (hPos < 0) {
            hPos = 0;
        }
        ps2Last = gamepad2.ps;
        slideLAngle.setPosition(pos);
        slideRAngle.setPosition(pos);
        clawHAngle.setPosition(hPos);
        clawVAngle.setPosition(vPos);

        if (gamepad2.y) {
            hangLeft.setPower(-1);
            hangRight.setPower(-1);
        } else if (gamepad2.a) {
            hangLeft.setPower(1);
            hangRight.setPower(1);
        } else {
            hangLeft.setPower(0);
            hangRight.setPower(0);
        }
        /*
        if (gamepad1.dpad_up) {
            clawVAngle.setPosition(clawVAngle.getPosition() + 0.005);
        }
        if (gamepad1.dpad_down) {
            clawVAngle.setPosition(clawVAngle.getPosition() - 0.005);
        }
        if (gamepad1.dpad_right) {
            clawHAngle.setPosition(clawHAngle.getPosition() - 0.005);
        }
        if (gamepad1.dpad_left) {
            clawHAngle.setPosition(clawHAngle.getPosition() + 0.005);
        )
        */

        telemetry.addData("slidePos", slidePos);
        telemetry.addData("halfSpeed", halfSpeedToggle);
        telemetry.addData("drivingReverse", drivingReverse);
        telemetry.addData("clawHAngle", clawHAngle.getPosition());
        telemetry.addData("clawVAngle", clawVAngle.getPosition());
        telemetry.addData("slideAngle", slideLAngle.getPosition());
        updateTelemetry(telemetry);

        drive.moveInTeleop(xMovement, yMovement, rotation, drivePower);

    }

}