package org.firstinspires.ftc.teamcode.LevineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdoWheelSetUp {
//    public double leftEncoderPos;
//    public double rightEncoderPos;
//    public double centerEncoderPos;
//    public double ogLeftEncoderPos;
//    public double ogRightEncoderPos;
//    public double ogCenterEncoderPos;
    public Pose2d startingPose;
    public Pose2d currPose;
    private LinearOpMode myOpMode;
    Telemetry telemetry;

    public OdoWheelSetUp(double leftEPos, double rightEPos, double centerEPos, LinearOpMode opMode){
//        leftEncoderPos = leftEPos;
//        rightEncoderPos = rightEPos;
//        centerEncoderPos = centerEPos;
//        ogLeftEncoderPos = leftEPos;
//        ogRightEncoderPos = rightEPos;
//        ogCenterEncoderPos = centerEPos;
        startingPose = new Pose2d(0, 0, Math.toRadians(0));
        currPose = startingPose;
        myOpMode = opMode;
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    public OdoWheelSetUp(double leftEPos, double rightEPos, double centerEPos, Pose2d startingPose, LinearOpMode opMode){
//        leftEncoderPos = leftEPos;
//        rightEncoderPos = rightEPos;
//        centerEncoderPos = centerEPos;
//        ogLeftEncoderPos = leftEPos;
//        ogRightEncoderPos = rightEPos;
//        ogCenterEncoderPos = centerEPos;
        this.startingPose = startingPose;
        currPose = startingPose;
        myOpMode = opMode;
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
    }

//    public OdoWheelNewTicks getTicksBetween(Pose2d currPose, Pose2d nextPose){
//        double ticksDist = (Math.sqrt(Math.pow(currPose.getX() - nextPose.getX(), 2) + Math.pow(currPose.getY() - nextPose.getY(), 2)))/LevineLocalizationMap.ticksToInches;
//        double deltaXTicks = ((currPose.getX() + nextPose.getX())/LevineLocalizationMap.ticksToInches)*LevineLocalizationMap.xMultiplier;
//        double deltaYTicks = ((currPose.getY() + nextPose.getY())/LevineLocalizationMap.ticksToInches)*LevineLocalizationMap.yMultiplier;
//        double deltaHeading = currPose.getHeading() + nextPose.getHeading();
//        double deltaHeadingTicksX = (deltaHeading*(LevineLocalizationMap.trackWidth/2))/LevineLocalizationMap.ticksToInches;
//        double deltaHeadingTicksY = (deltaHeading*(LevineLocalizationMap.centerWheelOffset/2))/LevineLocalizationMap.ticksToInches;
//        double newLeftTicks = -deltaHeadingTicksX + deltaXTicks + leftEncoderPos;
//        double newRightTicks = deltaHeadingTicksX + deltaXTicks + rightEncoderPos;
//        double newCenterTicks = deltaHeadingTicksY + deltaYTicks - centerEncoderPos;
//        return new OdoWheelNewTicks(newLeftTicks, newRightTicks, newCenterTicks);
//    }

//    public void updateTicksManual(double leftEPos, double rightEPos, double centerEPos){
//        leftEncoderPos = leftEPos;
//        rightEncoderPos = rightEPos;
//        centerEncoderPos = centerEPos;
//    }

//    public void updateTicksManual(OdoWheelNewTicks o){
//        leftEncoderPos = o.leftOdoWheelTick;
//        rightEncoderPos = o.rightOdoWheelTick;
//        centerEncoderPos = o.centerOdoWheelTick;
//    }
//    public void updateCurrPose(){
//        double difInLeftE = ((leftEncoderPos - ogLeftEncoderPos)* LevineLocalizationMap.xMultiplier);
//        double difInRightE = ((rightEncoderPos - ogRightEncoderPos)*LevineLocalizationMap.xMultiplier);
//        double difInCenterE = ((centerEncoderPos - ogCenterEncoderPos)*LevineLocalizationMap.yMultiplier);
//
//        double xMovement = (difInRightE*(LevineLocalizationMap.trackWidth/2) + difInLeftE*(LevineLocalizationMap.trackWidth/2)) / (LevineLocalizationMap.trackWidth);
//        double theta = (difInLeftE - difInRightE) / (LevineLocalizationMap.trackWidth);
//        double yMovement = difInCenterE - (LevineLocalizationMap.centerWheelOffset * theta);
//        double xPosNew = xMovement*LevineLocalizationMap.ticksToInches;
//        double yPosNew = yMovement*LevineLocalizationMap.ticksToInches;
//
//        currPose = new Pose2d(xPosNew, yPosNew, 2*Math.toRadians(theta) + Math.toDegrees(startingPose.getHeading()));
//    }
    public Pose2d getCurrPose(){
        return currPose;
    }

//    public OdoWheelNewTicks getCurrentTicks(){
//        return new OdoWheelNewTicks(leftEncoderPos, rightEncoderPos, centerEncoderPos);
//    }

    public WheelPowers getPowers(Pose2d currPose, Pose2d targetPose) {
        return getPowers(currPose, targetPose, LevineLocalizationMap.speedMultiplier);
    }
    public WheelPowers getPowers(Pose2d currPose, Pose2d targetPose, double speedMultiplier) {
        //Get dists
        double xDist = targetPose.getX() - currPose.getX();
        double yDist = targetPose.getY() - currPose.getY();
        double totAngDist = targetPose.getHeading() - currPose.getHeading();

        double distToTarget = Math.hypot(xDist, yDist);

        //get theta
        double theta = Math.atan2(yDist, xDist) + Math.toRadians(90) - targetPose.getHeading();

        telemetry.addLine("Theta is " + Math.toDegrees(theta));


        double ADPower = speedMultiplier * (Math.sin(theta) + Math.cos(theta));
        double BCPower = speedMultiplier * (Math.sin(theta) - Math.cos(theta));

        double turnPower = MathsAndStuff.AngleWrap(totAngDist);
        double turningScale = Math.max(Math.abs(ADPower + turnPower), Math.abs(ADPower - turnPower));
        turningScale = Math.max(turningScale, Math.max(Math.abs(BCPower + turnPower), Math.abs(BCPower - turnPower)));

        if (Math.abs(turningScale) < 1.0){
            turningScale = 1.0;
        }

        double fl = (ADPower - turnPower) / turningScale;
        double fr = (BCPower + turnPower) / turningScale;
        double bl = (BCPower - turnPower) / turningScale;
        double br = (ADPower + turnPower) / turningScale;

        return new WheelPowers(fr, fl, br, bl);
    }
}
