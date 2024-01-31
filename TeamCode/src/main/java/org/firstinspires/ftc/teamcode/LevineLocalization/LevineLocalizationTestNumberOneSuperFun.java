//package org.firstinspires.ftc.teamcode.LevineLocalization;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.hardwareMaps.MecanumDrive;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@TeleOp(group = "Levine Local")
//@Disabled
//public class LevineLocalizationTestNumberOneSuperFun extends LinearOpMode {
//    LevineLocalizationMap wMap = new LevineLocalizationMap(this);
//    boolean xDone = false;
//    boolean yDone = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        double driveSpeed = 1;
//        boolean drivingReversed = false;
//        wMap.init();
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        Pose2dLev firstPose = new Pose2dLev(0, 0, -90);
////        Pose2d currPose = new Pose2d(firstPose.getXPos(), firstPose.getYPos(), firstPose.getHeading());
//        Pose2dLev targetPose = new Pose2dLev(0, 20, 0);
//        MecanumDrive driveController = new MecanumDrive(wMap.frontLeft, wMap.frontRight, wMap.backLeft, wMap.backRight);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        waitForStart();
//
//        while(opModeIsActive()){
//            wMap.updateTicksAuto();
//            wMap.updatePose();
//            drive.update();
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            double xDist = targetPose.xPos - wMap.odos.currPose.getX();
//            double yDist = targetPose.yPos - wMap.odos.currPose.getY();
//            double angDist = targetPose.heading - wMap.odos.currPose.getHeading();
//
//            double distToTarget = Math.hypot(xDist, yDist);
//            double theta = MathsAndStuff.AngleWrap(Math.atan2(xDist, yDist) + wMap.odos.startingPose.getHeading());
//
//            //Error X
//            double relDistX = Math.cos(theta) * distToTarget;
//            double relErrorX = (Math.cos(theta) * LevineLocalizationMap.poseError);
//
//            double relDistY = Math.sin(theta) * distToTarget;
//            double relErrorY = (Math.sin(theta) * LevineLocalizationMap.poseError);
//            if (Math.abs(relDistX) < Math.abs(relErrorX)) {
//                xDone = true;
//            }
//            if (Math.abs(relDistY) < Math.abs(relErrorY)) {
//                yDone = true;
//            }
//
//            double difInLeftE = ((wMap.odos.leftEncoderPos - wMap.odos.ogLeftEncoderPos)* LevineLocalizationMap.xMultiplier);
//            double difInRightE = ((wMap.odos.rightEncoderPos - wMap.odos.ogRightEncoderPos)*LevineLocalizationMap.xMultiplier);
//            double difInCenterE = ((wMap.odos.centerEncoderPos - wMap.odos.ogCenterEncoderPos)*LevineLocalizationMap.yMultiplier);
//            double theta2 = (2*Math.toRadians((difInLeftE - difInRightE) / (LevineLocalizationMap.trackWidth)));
//            double difInTheta = theta2 - Math.toDegrees(wMap.odos.startingPose.getHeading());
////            double theta3 = 2*((difInLeftE - difInRightE) / (LevineLocalizationMap.trackWidth));
//            double theta4 = ((difInLeftE - difInRightE) / (LevineLocalizationMap.trackWidth));
//
//            double stickLx = this.gamepad1.left_stick_x;
//            double stickLy = this.gamepad1.left_stick_y;
//            double stickRx = this.gamepad1.right_stick_x;
//            boolean rb = this.gamepad1.right_bumper;
//            boolean lb = this.gamepad1.left_bumper;
//            boolean a = this.gamepad1.a;
//
//            if (rb) {
//                driveSpeed = 0.3;
//            } else if (lb) {
//                driveSpeed = 0.15;
//            }
//            else{
//                driveSpeed = 1;
//            }
//            if (drivingReversed) {
//                driveController.moveInTeleop(-stickLx, -stickLy, stickRx, driveSpeed);
//            } else {
//                driveController.moveInTeleop(stickLx, stickLy, stickRx, driveSpeed);
//            }
//
//            telemetry.addLine("CurrPose: " + wMap.odos.currPose);
//            telemetry.addLine("CurrLeftE: " + wMap.leftEncoder.getCurrentPosition());
//            telemetry.addLine("CurrRightE: " + wMap.rightEncoder.getCurrentPosition());
//            telemetry.addLine("CurrCenterE: " + wMap.centerEncoder.getCurrentPosition());
//            telemetry.addLine("Dist to target X: " + relDistX + " Dist to target Y: " + relDistY);
//            telemetry.addLine("Error X: " + relErrorX + " Error Y: " + relErrorY);
//            telemetry.addLine("X done? " + xDone + " Y done? " + yDone);
//            telemetry.addLine("Theta is " + (theta2));
//            telemetry.addLine("Weird ass math with theta is " + (LevineLocalizationMap.centerWheelOffset * theta2));
//            telemetry.addLine("Change In Theta Is " + difInTheta);
//            telemetry.addLine("Weird ass math with Dif In theta is " + (LevineLocalizationMap.centerWheelOffset * difInTheta));
////            telemetry.addLine("theta 3 is " + theta3);
////            telemetry.addLine("theta 4 is " + theta4);
//            telemetry.addLine("Weird ass math with theta4 is " + (LevineLocalizationMap.centerWheelOffset * theta4));
//            telemetry.addLine("Weird ass math with theta4 and dif is " + (difInCenterE - (LevineLocalizationMap.centerWheelOffset * theta4)));
//            telemetry.addLine("Dif in center E " + difInCenterE);
////            telemetry.addLine("Weird ass math with theta3 is " + (LevineLocalizationMap.centerWheelOffset * theta3));
////            telemetry.addLine("Weird ass math with theta4 is " + (LevineLocalizationMap.centerWheelOffset * theta4));
//
////            telemetry.addLine("Wheel Powers FL: " + wMap.frontLeft.getPower() + " FR: " + wMap.frontRight.getPower() + " BR: " + wMap.backRight.getPower() + " BL: " + wMap.backLeft.getPower());
////            telemetry.addLine("Wheel Directions FL: " + wMap.frontLeft.getDirection() + " FR: " + wMap.frontRight.getDirection() + " BR: " + wMap.backRight.getDirection() + " BL: " + wMap.backLeft.getDirection());
//            telemetry.update();
//        }
//    }
//}
