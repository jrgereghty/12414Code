package org.firstinspires.ftc.teamcode;

//luh Teleop
//default position pos = 0.9, vpos = 1

/*flat positions:
pos = 0 (70 deg), vpos = 0.205, slidePos = 0.299
pos = 0.05 (72 deg), vpos = 0.21, slidePos = 0.421
pos = 0.1 (75 deg), vpos = 0.214, slidePos = 0.573
*/

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.JayMap;
import org.firstinspires.ftc.teamcode.util.AprilTagAutoAlignmentManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "purple")
@Config

public class TeleOp extends OpMode {

    JayMap jayBot = new JayMap(this);
    MecanumDrive drive;


    AprilTagAutoAlignmentManager hPosCheck = new AprilTagAutoAlignmentManager();


    @Override
    public void init() {
        jayBot.init();
        drive = new MecanumDrive(jayBot.frontLeft, jayBot.frontRight, jayBot.backLeft, jayBot.backRight);
    }

    @Override
    public void loop() {

        jayBot.xMovement = gamepad1.left_stick_x;
        jayBot.yMovement = gamepad1.left_stick_y;
        jayBot.rotation = gamepad1.right_stick_x;
        jayBot.drivePower = Math.max(Math.max(Math.abs(jayBot.yMovement), Math.abs(jayBot.xMovement)), Math.abs(jayBot.rotation));

        if (!jayBot.aLast && gamepad1.a) {
            jayBot.halfSpeedToggle = !jayBot.halfSpeedToggle;
        }
        if (jayBot.halfSpeedToggle) {
            jayBot.drivePower *= 0.5;
        }
        jayBot.aLast = gamepad1.a;

        if (!jayBot.yLast && gamepad1.y) {
            jayBot.drivingReverse = !jayBot.drivingReverse;
        }
        if (jayBot.drivingReverse) {
            jayBot.xMovement *= -1;
            jayBot.yMovement *= -1;
        }
        jayBot.yLast = gamepad1.y;

        if (!jayBot.rightBumper2Last && gamepad2.right_bumper) {
            jayBot.clawROpen = !jayBot.clawROpen;
        }
        if (jayBot.clawROpen) {
            jayBot.openRightClaw();
        } else {
            jayBot.closeRightClaw();
        }
        jayBot.rightBumper2Last = gamepad2.right_bumper;

        if (!jayBot.leftBumper2Last && gamepad2.left_bumper) {
            jayBot.clawLOpen = !jayBot.clawLOpen;
        }
        if (jayBot.clawLOpen) {
            jayBot.openLeftClaw();
        } else {
            jayBot.closeLeftClaw();
        }
        jayBot.leftBumper2Last = gamepad2.left_bumper;

        jayBot.slidePos = jayBot.slide.getCurrentPosition() / 537.7 * 4 * Math.PI / 41.1;
        jayBot.slideLength = jayBot.slidePos * 41.1 + 38.5;//outputs slide position in cm
        if (gamepad2.right_trigger > 0.01) {
            jayBot.slidePower = jayBot.getSlideVelocity(1, jayBot.slidePos, Math.pow(gamepad2.right_trigger, 3));
            jayBot.slide.setPower(jayBot.slidePower);
        } else if (gamepad2.left_trigger > 0.01) {
            jayBot.slidePower = jayBot.getSlideVelocity(-1, jayBot.slidePos, Math.pow(gamepad2.left_trigger, 3));
            jayBot.slide.setPower(jayBot.slidePower);
        } else {
            if (jayBot.slidePos <= 0.08) {
                jayBot.slide.setPower(-0.5);
            } else if (jayBot.slidePos >= 0.92) {
                jayBot.slide.setPower(0.5);
            }
            jayBot.slide.setPower(0);
        }

        if (!jayBot.ps2Last && gamepad2.ps) {
            jayBot.intake = !jayBot.intake;
            jayBot.hPos = 0.5;
            if (jayBot.intake) {
                jayBot.pos = 0;
                jayBot.vPos = JayMap.getClawVAngle1(jayBot.slideLength);
            } else {
                jayBot.pos = 0.5;
                jayBot.vPos = JayMap.getClawVAngle2(jayBot.pos);
            }
        }
        if (jayBot.intake) {
            jayBot.pos = JayMap.getSlideAngle(jayBot.slideLength);
            jayBot.vPos = JayMap.getClawVAngle1(jayBot.slideLength);
        } else {
            if (gamepad2.left_stick_y >= 0.005 && jayBot.pos <= 0.99) {
                jayBot.pos += gamepad2.left_stick_y * 0.01;
            }
            if (gamepad2.left_stick_y <= -0.005 && jayBot.pos >= 0.01) {
                jayBot.pos += gamepad2.left_stick_y * 0.01;
            }
            jayBot.vPos = JayMap.getClawVAngle2(jayBot.pos);
        }
        if (gamepad2.right_stick_x >= 0.05) {
            jayBot.hPos += -gamepad2.right_stick_x * 0.01;
        }
        if (gamepad2.right_stick_x <= -0.05) {
            jayBot.hPos += -gamepad2.right_stick_x * 0.01;
        }
        /*
        if (gamepad2.start) {
           double newHPos = AprilTagAutoAlignmentManager.calculateAngleToTag(jayBot.bonoboCam);
           jayBot.setToAngle(newHPos);
        }

         */


        if (jayBot.hPos > 1) {
            jayBot.hPos = 1;
        } else if (jayBot.hPos < 0) {
            jayBot.hPos = 0;
        }
        if (jayBot.pos > 1) {
            jayBot.pos = 1;
        } else if (jayBot.pos < 0) {
            jayBot.pos = 0;
        }
        if (jayBot.vPos > 1) {
            jayBot.vPos = 1;
        } else if (jayBot.vPos < 0) {
            jayBot.vPos = 0;
        }

        jayBot.ps2Last = gamepad2.ps;
        jayBot.slideLAngle.setPosition(jayBot.pos);
        jayBot.slideRAngle.setPosition(jayBot.pos);
        jayBot.clawHAngle.setPosition(jayBot.hPos);
        jayBot.clawVAngle.setPosition(jayBot.vPos);

        if (gamepad2.y) {
            jayBot.hangLeft.setPower(-1);
            jayBot.hangRight.setPower(-1);
        } else if (gamepad2.a) {
            jayBot.hangLeft.setPower(1);
            jayBot.hangRight.setPower(1);
        } else {
            jayBot.hangLeft.setPower(0);
            jayBot.hangRight.setPower(0);
        }

        if (!jayBot.bLast && gamepad1.b){
            jayBot.planeLaunch = !jayBot.planeLaunch;
        }
        if (jayBot.planeLaunch) {
            jayBot.plane.setPosition(1);
        } else {
            jayBot.plane.setPosition(0);
        }
        jayBot.bLast = gamepad1.b;

        telemetry.addData("slidePos", jayBot.slidePos);
        telemetry.addData("halfSpeed", jayBot.halfSpeedToggle);
        telemetry.addData("drivingReverse", jayBot.drivingReverse);
        telemetry.addData("clawHAngle", jayBot.clawHAngle.getPosition());
        telemetry.addData("clawVAngle", jayBot.clawVAngle.getPosition());
        telemetry.addData("slideAngle", jayBot.slideLAngle.getPosition());
        telemetry.update();

        drive.moveInTeleop(jayBot.xMovement, jayBot.yMovement, jayBot.rotation, jayBot.drivePower);

    }

}