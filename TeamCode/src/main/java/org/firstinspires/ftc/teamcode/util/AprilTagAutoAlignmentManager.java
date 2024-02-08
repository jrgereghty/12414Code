package org.firstinspires.ftc.teamcode.util;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagAutoAlignmentManager {
    public static double calculateAngleToTag(AprilTagDetection detection, double servoCurrentAngle){

        double distanceToTag = detection.ftcPose.y;

        double horizontalDisplacement = detection.ftcPose.x;

        double maxServoAngle = 180;

        double angleToTag = Math.toDegrees(Math.atan(horizontalDisplacement / distanceToTag));

        double absoluteAngleToTag = servoCurrentAngle + angleToTag;

        absoluteAngleToTag = Math.max(-maxServoAngle, Math.min(maxServoAngle, absoluteAngleToTag));

        return absoluteAngleToTag;
    }

}
