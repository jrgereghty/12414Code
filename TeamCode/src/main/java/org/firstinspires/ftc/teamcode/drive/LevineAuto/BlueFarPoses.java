package org.firstinspires.ftc.teamcode.drive.LevineAuto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BlueFarPoses {
    //Starting pos
    public static double xPosStartingPos = -33, yPosStartingPos = 58;

    //After only poses
    public static double xPosPurplePixelPlacementAfterBeacon1 = 30.4, yPosPurplePixelPlacementAfterBeacon1 = 30;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 22.4, yPosPurplePixelPlacementAfterBeacon23 = 30;
    public static double xPosStartExtendFirstPlacementAfter = 40, yPosStartExtendFirstPlacementAfter = 35;
    public static double xPosFirstPlacementAfter = 46, yPosFirstPlacementAfterBeacon1 = 45, yPosFirstPlacementAfterBeacon2 = 35, yPosFirstPlacementAfterBeacon3 = 27;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -28, yPosGoAcrossForBeforeTrussPurplePixelFar = 10;
    public static double xPosGoAroundPurplePixelBeacon2 = -50, yPosGoAroundPurplePixelBeacon2 = 10;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -33, yPosGoAcrossForBeforeTrussPurplePixelClose = 44;

    //Far poses
    public static double xPosLineUpForPickUpFar = 40, yPosLineUpForPickUpFar = 9;
    public static double xPosLineUpForPlaceFar = 15, yPosLineUpForPlaceFar = 9;
    public static double xPosStartArmExtendPickUpFar = -21, yPosStartArmExtendPickUpFar = 9;
    public static double xPosPickUpPixelFar = -43, yPosPickUpPixelFar = 12;
    public static double xPosPlacePixelFar = 45, yPosPlacePixelFar = 18;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -16;
    public static double xPosStartArmExtendPlaceFar = 30, yPosStartArmExtendPlaceFar = 20;

    //Close poses
    public static double xPosLineUpForPickUpClose = 13, yPosLineUpForPickUpClose = 54.5;
    public static double xPosStartArmExtendPickUpClose = 0, yPosStartArmExtendPickUpClose = -52;
    public static double xPosPickUpPixelClose = -43, yPosPickUpPixelClose = 44;
    public static double xPosStartArmExtendPlaceClose = 30, yPosStartArmExtendPlaceClose = 54.5;
    public static double xPosPlacePixelClose = 47, yPosPlacePixelClose = 42;
    public static double xPosFlipAfterPlaceClose = 20, yPosFlipAfterPlaceClose = -42;
    public static double xPosGoStraightThroughTrussClose = -40, yPosGoStraightThroughTrussClose = 54.5;
    public static double xPosMidStackPickUpFar = -40, yPosMidStackPickUpFar = 13;
    public static double xPosMidStackPickUpClose = -50, yPosMidStackPickUpClose = 50;

    //Park Poses
    public static double xPosParkTriangle = 53, yPosParkTriangle = 10;
    public static double xPosLineUpParkTriangle = 40, yPosLineUpParkTriangle = 10;
    public static double xPosParkSquare = 53, yPosParkSquare = 57;
    public static double xPosLineUpParkSquare = 40, yPosLineUpParkSquare = 55;

    public static double headingMidStackPickUpFar = Math.toRadians(150), headingMidStackPickUpClose = Math.toRadians(230);
    public static double headingStartingPositionAndBeacon = -Math.toRadians(90), headingPickUpClose = Math.toRadians(215);
    public static double headingPlaceFar = Math.toRadians(200), headingPlaceClose = Math.toRadians(160);
    public static double headingWallBeaconBefore = Math.toRadians(245), headingTrussBeaconBefore = Math.toRadians(290), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(280), headingBeacon1PlacementAfter = Math.toRadians(195), headingBeacon3PlacementAfter = Math.toRadians(161);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(220), headingBeacon2PlacementBeforeClose = Math.toRadians(200), headingBeacon3PlacementBeforeClose = Math.toRadians(190);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(220), headingBeacon2PlacementBeforeFar = Math.toRadians(200), headingBeacon3PlacementBeforeFar = Math.toRadians(190);
}
