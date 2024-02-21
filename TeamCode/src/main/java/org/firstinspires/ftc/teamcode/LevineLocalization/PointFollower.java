//package org.firstinspires.ftc.teamcode.LevineLocalization;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.Auton.ActionRunnerFirstIterationCenterStageBlueBottem;
//import org.firstinspires.ftc.teamcode.CenterStageImportantFiles.HardwareMaps.MonkeyMap;
//import org.firstinspires.ftc.teamcode.RoadrunnerStuff.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.VisionTesting.OpenCVGreatestColorTest;
//import org.openftc.easyopencv.OpenCvCamera;
//
//import java.util.ArrayList;
//
//@Config
//public class PointFollower {
//    LinearOpMode myOpMode;
//    OpMode myOperatorMode;
//    LevineLocalizationMap wMap;
//    Telemetry telemetry;
//    SampleMecanumDrive drive;
//    ActionRunnerFirstIterationCenterStageBlueBottem actionRunner;
//    public ElapsedTime runtime = new ElapsedTime();
//    public ElapsedTime velTime = new ElapsedTime();
//    public static double isBuggingRuntimeToStop;
//    public static double isBuggingRuntimeToStopError = 0.5;
//    public static int finalCounter = 0;
//    public static int endCounter = 0;
//    public static int almostDoneCounter = 0;
//    int endOfPointCounter = 0;
//    ArrayList <PosesAndActions> posesToGoTo = new ArrayList<>();
//    ArrayList<String> trajTypes = new ArrayList<>();
//    ArrayList<PointType> pointTypes = new ArrayList<>();
//    ArrayList<Pose2d> inBetweenPoints = new ArrayList<>();
//    ArrayList<PointType> pointTypesInBetween = new ArrayList<>();
//    boolean xDone = false, yDone = false, angDone = false;
//    int donePointCounter = 0;
//    Pose2d startOfNewGo = new Pose2d(), prevPoseForVel = new Pose2d();
//    double currVelocity = 0;
//    public double currPower;
//    public double distToTarg;
//    public double totalPathDistance;
//    public static double maxVel = 50, almostDoneVel = 20, slowVel = almostDoneVel, evenSlowerVel = 2, testVel = 20, slowestVel = 5, velRoomForErrorIsBuggingTimer = 3;
//    public static double decceleration = 100;
//    public double targetVelocity = maxVel;
//    public static double accelerationConst = 200;
//    public static PIDCoefficients PIDVals = new PIDCoefficients(0.23, 0, 0.5);
//
//    public PointFollower(LinearOpMode opmode, ActionRunnerFirstIterationCenterStageBlueBottem actionRunner) {
//        myOpMode = opmode;
//        wMap = new LevineLocalizationMap(this.myOpMode);
//        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
//        this.actionRunner = actionRunner;
//    }
//
//    public void init(ArrayList<PosesAndActions> posesToGoTo, boolean isTest) {
//        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
//        if(isTest){
//            maxVel = testVel;
//            slowVel = testVel;
//            almostDoneVel = testVel;
//        }
//
//        wMap.init(posesToGoTo.get(0).pose);
//        startOfNewGo = posesToGoTo.get(0).pose;
//        drive = new SampleMecanumDrive(this.myOpMode.hardwareMap);
//        drive.setPoseEstimate(posesToGoTo.get(0).pose);
//
//        this.posesToGoTo.clear();
//        trajTypes.clear();
//        pointTypes.clear();
//        inBetweenPoints.clear();
//        pointTypesInBetween.clear();
//
//        this.posesToGoTo.addAll(posesToGoTo);
//
//        for (int i = 1; i < this.posesToGoTo.size(); i++) {
//            trajTypes.add("");
//        }
//        for (int i = 0; i < this.posesToGoTo.size(); i++) {
//            pointTypes.add(new PointType("mid"));
//        }
//        pointTypes.set(pointTypes.size() - 1, new PointType("end"));
//
//        double theoreticalTheta;
//        for (int i = 1; i < this.posesToGoTo.size(); i++) {
//            Pose2d startingPos = this.posesToGoTo.get(i - 1).pose;
//            Pose2d targetPos = this.posesToGoTo.get(i).pose;
//
//            double xDist = targetPos.getX() - startingPos.getX();
//
//            double yDist = targetPos.getY() - startingPos.getY();
//
//            double totDistToTarget = Math.hypot(xDist, yDist);
//
//            //Iteration for poses calculator
//            int iterationsForPoses = (int) ((totDistToTarget / 2) * LevineLocalizationMap.poseFollowCoef);
//
//            double distToTarget = totDistToTarget / iterationsForPoses;
//
//            distToTarg = distToTarget;
//
//
//            theoreticalTheta = MathsAndStuff.AngleWrap(Math.atan2(yDist, xDist));
//
//            for (int j = 0; j < iterationsForPoses; j++) {
//                double relDistX = Math.cos(theoreticalTheta) * (distToTarget * j);
//                double relDistY = Math.sin(theoreticalTheta) * (distToTarget * j);
//                inBetweenPoints.add(new Pose2d(startingPos.getX() + relDistX, startingPos.getY() + relDistY, targetPos.getHeading()));
//            }
//
//            //Get the cheek lengths
//            int leftPoints = (int) ((LevineLocalizationMap.followRadius / 2) * LevineLocalizationMap.poseFollowCoef);
//            int rightPoints;
//            if (pointTypes.get(i).type.equals("mid")) {
//                rightPoints = leftPoints;
//            } else {
//                rightPoints = 0;
//            }
//            if (leftPoints + rightPoints >= iterationsForPoses) {
//                leftPoints = iterationsForPoses / 2;
//                rightPoints = iterationsForPoses - leftPoints;
//            }
//
//            int midPoints = iterationsForPoses - (leftPoints + rightPoints);
//
//            for (int j = 0; j < leftPoints; j++) {
//                pointTypesInBetween.add(new PointType("mid"));
//            }
//            for (int j = 0; j < midPoints; j++) {
//                pointTypesInBetween.add(new PointType("inside"));
//            }
//            for (int j = 0; j < rightPoints; j++) {
//                pointTypesInBetween.add(new PointType("mid"));
//            }
//            pointTypesInBetween.set(pointTypesInBetween.size() - 1, new PointType("endofpoint"));
//        }
//        for (int j = 3; j < 4; j++) {
//            if (pointTypesInBetween.size() - j > 0) {
//                pointTypesInBetween.set(pointTypesInBetween.size() - j, new PointType("almostdone"));
//            }
//        }
//        for (int j = 1; j < 3; j++) {
//            if (pointTypesInBetween.size() - j > 0) {
//                pointTypesInBetween.set(pointTypesInBetween.size() - j, new PointType("end"));
//            }
//        }
//        inBetweenPoints.add(new Pose2d(this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getX(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getY(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getHeading()));
//        pointTypesInBetween.add(new PointType("final"));
//
////        totalPathDistance = 0;
////        for (int i = 1; i < posesToGoTo.size(); i++) {
////            totalPathDistance += Math.abs(Math.hypot(posesToGoTo.get(i-1).pose.getX() - posesToGoTo.get(i).pose.getX(), posesToGoTo.get(i-1).pose.getY() - posesToGoTo.get(i).pose.getY()));
////        }
////        isBuggingRuntimeToStop = (Math.abs(totalPathDistance) / maxVel) + isBuggingRuntimeToStopError;
//
////        telemetry.addLine("weirdMath " + Math.abs(Math.hypot(posesToGoTo.get(0).pose.getX() - posesToGoTo.get(1).pose.getX(), posesToGoTo.get(0).pose.getY() - posesToGoTo.get(1).pose.getY())));
////        telemetry.addLine("isBuggingRuntimeToStop " + isBuggingRuntimeToStop);
//        telemetry.addLine("Poses to go to " + this.posesToGoTo);
//        telemetry.addLine("Poses in between " + inBetweenPoints);
//        telemetry.addLine("PointTypes in between: " + pointTypesInBetween);
//        telemetry.update();
//    }
//
//    public void reinit(ArrayList<PosesAndActions> posesToGoTo) {
//        posesToGoTo.add(0, new PosesAndActions(drive.getPoseEstimate(), ""));
//        init(posesToGoTo, false);
//        telemetry.addLine("poses to go to 1 is " + posesToGoTo.get(0));
//    }
//    public void goToPoints(boolean stopAfter){
//        goToPoints(stopAfter, maxVel);
//    }
//
//
//    public void goToPoints(boolean stopAfter, double newMaxVel) {
//        double distNeededToStartDecel = ((Math.pow(slowestVel, 2) - Math.pow(newMaxVel, 2))/(-2*decceleration));
//
//        totalPathDistance = 0;
//        for (int i = 1; i < posesToGoTo.size(); i++) {
//            totalPathDistance += Math.abs(Math.hypot(posesToGoTo.get(i-1).pose.getX() - posesToGoTo.get(i).pose.getX(), posesToGoTo.get(i-1).pose.getY() - posesToGoTo.get(i).pose.getY()));
//        }
//        isBuggingRuntimeToStop = (((totalPathDistance) / (newMaxVel / velRoomForErrorIsBuggingTimer)) + (((slowestVel - newMaxVel) / (decceleration)) * -1)) + isBuggingRuntimeToStopError;
//        runtime.reset();
//
//        posesToGoTo.remove(0);
//        startOfNewGo = drive.getPoseEstimate();
//        prevPoseForVel = drive.getPoseEstimate();
//        currPower = 0;
//        GetVelocityPIDController getVel = new GetVelocityPIDController(PIDVals, targetVelocity);
//        velTime.reset();
//        Pose2d currPose = drive.getPoseEstimate();
//
//        while (!(inBetweenPoints.isEmpty())) {
//            getVel.changeTarget(targetVelocity);
//            drive.update();
//
//            currPose = drive.getPoseEstimate();
//
//            double timeForVel = velTime.seconds();
//
//            double totDistForVel = Math.hypot(currPose.getX() - prevPoseForVel.getX(), currPose.getY() - prevPoseForVel.getY());
//            currVelocity = Math.abs(totDistForVel / timeForVel);
//
//            if (!inBetweenPoints.isEmpty()) {
//                double isBuggingChecker = runtime.seconds();
//                Pose2d targetPose = inBetweenPoints.get(0);
//                double roomForPoseError = new PointType("mid").followRadius / 2;
//                if (stopAfter){
//                    roomForPoseError = pointTypesInBetween.get(0).followRadius / 2;
//                }
//                else{
//                    roomForPoseError = new PointType("mid").followRadius / 2;
//                }
//
//                angDone = !pointTypesInBetween.get(0).type.equals("final");
//
//                double xDist = targetPose.getX() - currPose.getX();
//                double yDist = targetPose.getY() - currPose.getY();
//                double angDist = targetPose.getHeading() - currPose.getHeading();
//
//                double distToTarget = Math.hypot(xDist, yDist);
//                double theta = MathsAndStuff.AngleWrap(Math.atan2(xDist, yDist) + wMap.startingPose.getHeading());
//
//                double totXDist = posesToGoTo.get(posesToGoTo.size()-1).pose.getX() - currPose.getX();
//                double totYDist = posesToGoTo.get(posesToGoTo.size()-1).pose.getY() - currPose.getY();
//                double totDistToTarget = Math.hypot(totXDist, totYDist);
//                //Error X
//                double relDistX = Math.cos(theta) * distToTarget;
//                double relErrorX = (Math.cos(theta) * roomForPoseError);
//
//                double relDistY = Math.sin(theta) * distToTarget;
//                double relErrorY = (Math.sin(theta) * roomForPoseError);
//
//                if (Math.abs(relDistX) < Math.abs(relErrorX)) {
//                    xDone = true;
//                }
//                if (Math.abs(relDistY) < Math.abs(relErrorY)) {
//                    yDone = true;
//                }
//
//                if (stopAfter){
//                    if(Math.abs(totDistToTarget) < Math.abs(distNeededToStartDecel)){
//                        targetVelocity = targetVelocity - (decceleration*timeForVel);
//                    }
//                    else{
//                        targetVelocity = newMaxVel;
//                    }
//                }
//                else{
//                    targetVelocity = newMaxVel;
//                }
//                if(targetVelocity < slowestVel){
//                    targetVelocity = slowestVel;
//                }
//                if (currVelocity < targetVelocity || currVelocity > targetVelocity) {
//                    currPower += getVel.calculate(currVelocity) / accelerationConst;
//                }
//                if (currPower < 0) {
//                    currPower = 0;
//                }
//                if (currPower > 1) {
//                    currPower = 1;
//                }
//
//                wMap.setMotorPowers(currPose, targetPose, currPower);
//
//                if (isBuggingChecker > isBuggingRuntimeToStop) {
//                    xDone = true;
//                    yDone = true;
//                    angDone = true;
//                }
//                if (Math.abs(angDist) < Math.abs(LevineLocalizationMap.angError)) {
//                    angDone = true;
//                }
//
//                prevPoseForVel = currPose;
//                velTime.reset();
//
//                if (xDone && yDone && angDone) {
//                    if(pointTypesInBetween.get(0).type.equals("endofpoint")){
//                        actionRunner.runActions(posesToGoTo.get(0).action);
//                        posesToGoTo.remove(0);
//                        endOfPointCounter++;
//                    }
//                    inBetweenPoints.remove(0);
//                    pointTypesInBetween.remove(0);
//
//                    donePointCounter++;
//                    xDone = false;
//                    yDone = false;
//                    startOfNewGo = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getHeading());
//                }
//
//                telemetry.addData("Velocity ", currVelocity);
//                telemetry.addData("Tot dist to target ", totDistToTarget);
//                telemetry.addData("Dist needed for decel ", distNeededToStartDecel);
//                telemetry.addData("posesToGoTo ", posesToGoTo);
//                telemetry.addData("Curr Targ Vel: ", targetVelocity);
//                telemetry.addData("Curr Pose Error: ", roomForPoseError);
////                telemetry.addLine("Poses to go to " + posesToGoTo);
////                telemetry.addLine("Poses in between " + inBetweenPoints);
//                telemetry.addLine("Target Pose: " + targetPose);
//                telemetry.addLine("isBuggingChecker: " + isBuggingChecker);
////                telemetry.addLine("relDistX: " + relDistX + "relDistY: " + relDistY);
//                telemetry.addLine("Motor powers: " + wMap.getPowers(currPose, targetPose));
////                telemetry.addLine("Ang Done? " + angDone);
////                telemetry.addLine("X Done " + xDone);
////                telemetry.addLine("Y Done " + yDone);
////                telemetry.addLine("Done Points Counter: " + donePointCounter);
//                telemetry.addLine("CurrSpeed: " + currPower);
//                telemetry.addLine("distToTarget " + distToTarget);
//                telemetry.addData("totalPathDistance ", totalPathDistance);
//                telemetry.addLine("distToTarg " + distToTarg);
//                telemetry.addLine("is bugging time to stop is " + isBuggingRuntimeToStop);
//                telemetry.addLine("endofpoint counter " + endOfPointCounter);
//
////                telemetry.addLine("finalCounter: " + finalCounter);
////                telemetry.addLine("endCounter: " + endCounter);
////                telemetry.addLine("almostDoneCounter: " + almostDoneCounter);
//                telemetry.update();
//            }
//        }
//        if(stopAfter) {
//            wMap.stopMotors();
//        }
//    }
//}

package org.firstinspires.ftc.teamcode.LevineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;



@Config
public class PointFollower {
    LinearOpMode myOpMode;
    LevineLocalizationMap wMap;
    Telemetry telemetry;
    SampleMecanumDrive drive;
    ActionRunnerCenterStageAuton actionRunner;
    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime velTime = new ElapsedTime();
    public static double isBuggingRuntimeToStop;
    public static double isBuggingRuntimeToStopError = 2;
    int endOfPointCounter = 0;
    ArrayList <PosesAndActions> posesToGoTo = new ArrayList<>();
    ArrayList<PointType> pointTypes = new ArrayList<>();
    ArrayList<Pose2d> inBetweenPoints = new ArrayList<>();
    ArrayList<PointType> pointTypesInBetween = new ArrayList<>();
    boolean xDone = false, yDone = false, angDone = false;
    int donePointCounter = 0;
    Pose2d startOfNewGo = new Pose2d(), prevPoseForVel = new Pose2d();
    public double currVelocity = 0;
    public double currPower;
    public static double maxVel = 40, testVel = 20, slowestVel = 4;
    public static double decceleration = 40;
    public double targetVelocity = maxVel;
    public static double accelerationConst = 200;
    public static PIDCoefficients PIDVals = new PIDCoefficients(0.15, 0, 0.5);
    public int isBuggingCounter = 0;

    public PointFollower(LinearOpMode opmode, ActionRunnerCenterStageAuton actionRunner) {
        myOpMode = opmode;
        wMap = new LevineLocalizationMap(this.myOpMode);
        telemetry = new MultipleTelemetry(this.myOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.actionRunner = actionRunner;
    }

    public void init(ArrayList<PosesAndActions> posesToGoTo, boolean isTest, boolean firstTime) {
        if(isTest){
            maxVel = testVel;
        }

        if(firstTime){
            drive = new SampleMecanumDrive(this.myOpMode.hardwareMap);
        }

        wMap.init(posesToGoTo.get(0).pose);
        startOfNewGo = posesToGoTo.get(0).pose;
        drive.setPoseEstimate(posesToGoTo.get(0).pose);

        this.posesToGoTo.clear();
        pointTypes.clear();
        inBetweenPoints.clear();
        pointTypesInBetween.clear();

        this.posesToGoTo.addAll(posesToGoTo);

        for (int i = 0; i < this.posesToGoTo.size(); i++) {
            pointTypes.add(new PointType("mid"));
        }
        pointTypes.set(pointTypes.size() - 1, new PointType("end"));

        double theoreticalTheta;
        for (int i = 1; i < this.posesToGoTo.size(); i++) {
            Pose2d startingPos = this.posesToGoTo.get(i - 1).pose;
            Pose2d targetPos = this.posesToGoTo.get(i).pose;

            double xDist = targetPos.getX() - startingPos.getX();

            double yDist = targetPos.getY() - startingPos.getY();

            double totDistToTarget = Math.hypot(xDist, yDist);

            //Iteration for poses calculator
            int iterationsForPoses = (int) ((totDistToTarget / 2) * LevineLocalizationMap.poseFollowCoef);

            double distToTarget = totDistToTarget / iterationsForPoses;

            theoreticalTheta = MathsAndStuff.AngleWrap(Math.atan2(yDist, xDist));

            for (int j = 0; j < iterationsForPoses; j++) {
                double relDistX = Math.cos(theoreticalTheta) * (distToTarget * j);
                double relDistY = Math.sin(theoreticalTheta) * (distToTarget * j);
                inBetweenPoints.add(new Pose2d(startingPos.getX() + relDistX, startingPos.getY() + relDistY, targetPos.getHeading()));
            }
            for(int j = 0; j < iterationsForPoses; j++){
                pointTypesInBetween.add(new PointType("mid"));
            }
            pointTypesInBetween.set(pointTypesInBetween.size() - 1, new PointType("endofpoint"));
        }
        inBetweenPoints.add(new Pose2d(this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getX(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getY(), this.posesToGoTo.get(this.posesToGoTo.size() - 1).pose.getHeading()));
        pointTypesInBetween.add(new PointType("final"));

        telemetry.addLine("Poses to go to " + this.posesToGoTo);
        telemetry.addLine("Poses in between " + inBetweenPoints);
        telemetry.addLine("PointTypes in between: " + pointTypesInBetween);
        telemetry.update();
    }

    public void reinit(ArrayList<PosesAndActions> posesToGoTo) {
        drive.update();
        posesToGoTo.add(0, new PosesAndActions(drive.getPoseEstimate(), ""));
//        posesToGoTo.add(0, new PosesAndActions(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getRawExternalHeading()), ""));
        init(posesToGoTo, false, false);
//        telemetry.addLine("poses to go to 1 is " + posesToGoTo.get(0));
    }

    public void goToPoints(boolean stopAfter){
        goToPoints(stopAfter, maxVel);
    }

    public void goToPoints(boolean stopAfter, double newMaxVel) {
        startOfNewGo = drive.getPoseEstimate();
//        startOfNewGo = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getRawExternalHeading());
        prevPoseForVel = drive.getPoseEstimate();
//        prevPoseForVel = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getRawExternalHeading());
        currPower = 0;
        GetVelocityPIDController getVel = new GetVelocityPIDController(PIDVals, targetVelocity);
        velTime.reset();
        Pose2d currPose = drive.getPoseEstimate();
//        Pose2d currPose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getRawExternalHeading());
        targetVelocity = newMaxVel;
        double distNeededToStartDecel = ((Math.pow(slowestVel, 2) - Math.pow(newMaxVel, 2))/(-2*decceleration));
        double totXDist = 0;
        double totYDist = 0;
        for (int i = 1; i < posesToGoTo.size(); i++) {
            totXDist += Math.abs(posesToGoTo.get(i).pose.getX() - posesToGoTo.get(i-1).pose.getX());
            totYDist += Math.abs(posesToGoTo.get(i).pose.getY() - posesToGoTo.get(i-1).pose.getY());
        }
        double totDistToTarget = Math.hypot(totXDist, totYDist);
        double prevDistToTarget = totDistToTarget;

        if(distNeededToStartDecel > totDistToTarget){
            targetVelocity = Math.sqrt(Math.abs(Math.pow(slowestVel, 2) + (2*decceleration*totDistToTarget)));
        }
        posesToGoTo.remove(0);

        while (!(inBetweenPoints.isEmpty())) {
            getVel.changeTarget(targetVelocity);
            drive.update();

            currPose = drive.getPoseEstimate();
//            currPose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getRawExternalHeading());

            double timeForVel = velTime.seconds();

            double totDistForVel = Math.hypot(currPose.getX() - prevPoseForVel.getX(), currPose.getY() - prevPoseForVel.getY());
            currVelocity = Math.abs(totDistForVel / timeForVel);
            isBuggingRuntimeToStop = isBuggingRuntimeToStopError;

            if (!inBetweenPoints.isEmpty()) {
                prevDistToTarget = totDistToTarget;
                double isBuggingChecker = runtime.seconds();
                Pose2d targetPose = inBetweenPoints.get(0);
                double roomForPoseError = new PointType("mid").followRadius / 2;
                if (stopAfter){
                    roomForPoseError = pointTypesInBetween.get(0).followRadius / 2;
                }
                else{
                    roomForPoseError = new PointType("mid").followRadius / 2;
                }

                angDone = !pointTypesInBetween.get(0).type.equals("final");

                double xDist = targetPose.getX() - currPose.getX();
                double yDist = targetPose.getY() - currPose.getY();
                double angDist = targetPose.getHeading() - currPose.getHeading();

                double distToTarget = Math.hypot(xDist, yDist);
                double theta = MathsAndStuff.AngleWrap(Math.atan2(xDist, yDist) + wMap.startingPose.getHeading());

                if(posesToGoTo.size()-1 >= 0){
                    totXDist = Math.abs(posesToGoTo.get(posesToGoTo.size()-1).pose.getX() - currPose.getX());
                    totYDist = Math.abs(posesToGoTo.get(posesToGoTo.size()-1).pose.getY() - currPose.getY());
                }

                totDistToTarget = Math.hypot(totXDist, totYDist);

                //Error X
                double relDistX = Math.cos(theta) * distToTarget;
                double relErrorX = (Math.cos(theta) * roomForPoseError);

                double relDistY = Math.sin(theta) * distToTarget;
                double relErrorY = (Math.sin(theta) * roomForPoseError);

                if (Math.abs(relDistX) < Math.abs(relErrorX)) {
                    xDone = true;
                }
                if (Math.abs(relDistY) < Math.abs(relErrorY)) {
                    yDone = true;
                }

                if (stopAfter){
                    if(Math.abs(totDistToTarget) < Math.abs(distNeededToStartDecel) && totDistToTarget < prevDistToTarget + targetVelocity*timeForVel /*roomForErrorStartDecel*/){
                        targetVelocity = targetVelocity - (decceleration*timeForVel);
                    }
                    else if(Math.abs(totDistToTarget) > Math.abs(distNeededToStartDecel)){
                        targetVelocity = newMaxVel;
                    }
                }
                else{
                    targetVelocity = newMaxVel;
                }
                if(targetVelocity < slowestVel){
                    targetVelocity = slowestVel;
                }
                if (currVelocity < targetVelocity || currVelocity > targetVelocity) {
                    currPower += getVel.calculate(currVelocity) / accelerationConst;
                }
                if (currPower < 0) {
                    currPower = 0;
                }
                if (currPower > 1) {
                    currPower = 1;
                }

                wMap.setMotorPowers(currPose, targetPose, currPower);

                if (isBuggingChecker > isBuggingRuntimeToStop) {
                    xDone = true;
                    yDone = true;
                    angDone = true;
                    isBuggingCounter++;
                }
                if (Math.abs(angDist) < Math.abs(LevineLocalizationMap.angError)) {
                    angDone = true;
                }

                prevPoseForVel = currPose;
                velTime.reset();

                if (xDone && yDone && angDone) {
                    if(pointTypesInBetween.get(0).type.equals("endofpoint")){
                        actionRunner.runActions(posesToGoTo.get(0).action);
                        posesToGoTo.remove(0);
                        endOfPointCounter++;
                    }

                    runtime.reset();
                    inBetweenPoints.remove(0);
                    pointTypesInBetween.remove(0);

                    donePointCounter++;
                    xDone = false;
                    yDone = false;
                    startOfNewGo = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getHeading());
                }

                telemetry.addData("Velocity ", currVelocity);
                telemetry.addData("targetVelocity ", targetVelocity);
                telemetry.addData("New Max Vel ", newMaxVel);
                telemetry.addData("Tot dist to target ", totDistToTarget);
//                telemetry.addData("Dist needed for decel ", distNeededToStartDecel);
                telemetry.addData("posesToGoTo ", posesToGoTo);
//                telemetry.addLine("Target Pose: " + targetPose);
                telemetry.addLine("isBuggingChecker: " + isBuggingChecker);
//                telemetry.addLine("CurrSpeed: " + currPower);
//                telemetry.addLine("endofpoint counter " + endOfPointCounter);
//                telemetry.addData("Is Bugging Counter: ", isBuggingCounter);
                telemetry.update();
            }
        }
        if(stopAfter) {
            wMap.stopMotors();
        }
    }
}