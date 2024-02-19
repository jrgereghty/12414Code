package org.firstinspires.ftc.teamcode.util;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.EpicSuperCoolAprilTags;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagAutoAlignmentManager {
    //public static WebcamName cam1;
    public static double calculateAngleToTag(WebcamName cam1){//AprilTagDetection detection,
        EpicSuperCoolAprilTags aprilTag = new EpicSuperCoolAprilTags();

        aprilTag.AprilTagPro(cam1);


        List<AprilTagDetection> currentDetections = aprilTag.MythicalaprilTags();
        AprilTagDetection detection = currentDetections.get(1);
        aprilTag.telemetryAprilTag();

        double distanceToTag = detection.ftcPose.y;

        double horizontalDisplacement = detection.ftcPose.x;

        double tagYaw = detection.ftcPose.yaw;

        double tagBearing = detection.ftcPose.bearing;

        double maxServoAngle = 180;

        double angleToTag = Math.toDegrees(Math.atan(horizontalDisplacement / distanceToTag));//this should be equal to tag bearing
/*
        double absoluteAngleToTag = servoCurrentAngle + angleToTag;

        absoluteAngleToTag = Math.max(-maxServoAngle, Math.min(maxServoAngle, absoluteAngleToTag));

 */

        double absoluteAngleToTag = (90 - tagYaw)/180;



        return absoluteAngleToTag;
    }

}
