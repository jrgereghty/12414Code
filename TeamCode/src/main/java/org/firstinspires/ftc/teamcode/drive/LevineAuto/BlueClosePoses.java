package org.firstinspires.ftc.teamcode.drive.LevineAuto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BlueClosePoses {
    //Starting pos
    public static double sideMultiplierX = 1;
    public static double sideMultipliery = 1;
    public static double angleMultiplier = -1;
    public static double xPosStartingPos = 12 * sideMultiplierX, yPosStartingPos = 62 * sideMultipliery, headingStartingPos = Math.toRadians(270);
    public static double xPosStartingPos2 = 12 * sideMultiplierX, yPosStartingPos2 = 53 * sideMultipliery;

    //Purple Pixel Placement
    public static double xPosStartingPos3 = 12 * sideMultiplierX, yPosStartingPos3 = 53 * sideMultipliery, headingStartingPos3 = Math.toRadians(180);
    public static double xPosLeftSpikeMark = 24.5, yPosLeftSpikeMark = 55 * sideMultipliery, headingLeftSpikeMark = Math.toRadians(270);//x,40,y35,y = 180 Left
    //x=44.5,
    public static double xPosMiddleSpikeMark = 12, yPosMiddleSpikeMark = 49 * sideMultipliery, headingMiddleSpikeMark = Math.toRadians(270);
    public static double xPosRightSpikeMark = 27, yPosRightSpikeMark = 35 * sideMultipliery, headingRightSpikeMark = Math.toRadians(180);
    public static double xPosTurn2BoardMid = 12, yPosTurn2BoardMid = 49 * sideMultipliery, headingTurn2BoardMid = Math.toRadians(0);
    //x=20,y35, H = 180 Right
    //x=13 ,y=49, heading = 270 Mid


    //Poses For board Placing
    public static double xPosLeftBoardPlace = 44.5, yPosLeftBoardPlace = 42 * sideMultipliery;//revert after tuning, x value
    public static double xPosMidBoardPlace = 44.5, yPosMidBoardPlace = 33.5 * sideMultipliery;
    public static double xPosRightBoardPlace = 45.5, yPosRightBoardPlace = 29 * sideMultipliery;
    public static double PerpendicularBoardPlacementHeading = Math.toRadians(0);
    //x=44.5, 38.5, H=0 Mid 2
    public static double xPosBoardBack = 40, yPosBoardBack = 11 * sideMultipliery;

    //Middle 2+ Poses
    public static double xPosDoorLaneAlignmentBoard = 40, yPosDoorLaneAlignmentBoard = 14 * sideMultipliery, headingDoorLaneAlignmentBoard = Math.toRadians(180);//Return here after grabbing whites
    public static double xPosWhitePickupMid = -45, yPosWhitePickupMid = 14 * sideMultipliery, headingWhitePickupMid = Math.toRadians(180);

    //Placement from door poses and headings
    public static double xPosDoorSidePlace = 51, yPosDoorSidePlace = 20 * sideMultipliery;
    public static double headingRightDoorSidePlace = Math.toRadians(40)*angleMultiplier, headingMidDoorSidePlace = Math.toRadians(55)*angleMultiplier, headingLeftDoorSidePlace = Math.toRadians(57)*angleMultiplier;
    //Truss 2+ Poses
    public static double xPosTrussAlignmentBoard = 30, yPosTrussAlignmentBoard = 61 * sideMultipliery, headingTrussAlignmentBoard = Math.toRadians(180);
    public static double xPosTrussAlignmentFar = -39, yPosTrussAlignmentFar = 61 * sideMultipliery, headingTrussAlignmentFar = Math.toRadians(180);
    public static double xPosTrussWhite = -51, yPosTrussWhite = 50 * sideMultipliery, headingTrussWhite = -Math.toRadians(213);
    //Parking
    public static double xPosStartParkMid = 40, yPosStartParkMid = 11* sideMultipliery;
    public static double xPosEndParkMid = 53, yPosEndParkMid = 11* sideMultipliery;
    public static double xPosStartParkEdge = 40, yPosStartParkEdge = 60* sideMultipliery;
    public static double xPosEndParkEdge = 53, yPosEndParkEdge = 60 * sideMultipliery;
    public static double faceBoard = Math.toRadians(0), faceWhites = Math.toRadians(180);
    //Old Style purple placement:
    public static double xPosLine4StartLeft = 12, yPosLine4StartLeft = 53 * sideMultipliery, headingLine4StartLeft = Math.toRadians(307);
    public static double xPosLine4StartMid = 12, yPosLine4StartMid = 51 * sideMultipliery, headingLine4StartMid = Math.toRadians(270);
    public static double xPosLine4StartRight = 12, yPosLine4StartRight = 53 * sideMultipliery, headingLine4StartRight = Math.toRadians(245);
    public static double xPosReset4Left = 12, yPosReset4Left = 59 * sideMultipliery;//Heading is face board

    //After only poses LEVINE _______V_V_V_V_V_V_V_V_V_V_V_V_V_V_V_V_V_V______
    /*
    public static double xPosPurplePixelPlacementAfterBeacon1 = 38, yPosPurplePixelPlacementAfterBeacon1 = 30;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 28, yPosPurplePixelPlacementAfterBeacon23 = 27;
    public static double xPosStartExtendFirstPlacementAfter = 45, yPosStartExtendFirstPlacementAfter = 33;
    public static double xPosFirstPlacementAfter = 50, yPosFirstPlacementAfterBeacon1 = 40, yPosFirstPlacementAfterBeacon2 = 33, yPosFirstPlacementAfterBeacon3 = 23;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -36, yPosGoAcrossForBeforeTrussPurplePixelFar = -5;
    public static double xPosGoAroundPurplePixelBeacon2 = -38, yPosGoAroundPurplePixelBeacon2 = -15;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -36, yPosGoAcrossForBeforeTrussPurplePixelClose = -56;

    //Far poses
    public static double xPosLineUpForPickUpFar = 40, yPosLineUpForPickUpFar = 8;
    public static double xPosLineUpForPlaceFar = 15, yPosLineUpForPlaceFar = 8;
    public static double xPosStartArmExtendPickUpFar = -21, yPosStartArmExtendPickUpFar = 8;
    public static double xPosPickUpPixelFar = -41.5, yPosPickUpPixelFar = 10;
    public static double xPosPlacePixelFar = 47, yPosPlacePixelFar = 23;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -16;
    public static double xPosStartArmExtendPlaceFar = 35, yPosStartArmExtendPlaceFar = 8;

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
    public static double xPosParkSquare = 53, yPosParkSquare = 55;
    public static double xPosLineUpParkSquare = 40, yPosLineUpParkSquare = 55;

    public static double headingMidStackPickUpFar = Math.toRadians(150), headingMidStackPickUpClose = Math.toRadians(230);
    public static double headingStartingPositionAndBeacon = -Math.toRadians(90), headingPickUpClose = Math.toRadians(215);
    public static double headingPlaceFar = Math.toRadians(200), headingPlaceClose = Math.toRadians(160);
    public static double headingWallBeaconBefore = Math.toRadians(55), headingTrussBeaconBefore = Math.toRadians(125), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(100), headingBeacon1PlacementAfter = Math.toRadians(195), headingBeacon3PlacementAfter = Math.toRadians(161);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(60), headingBeacon2PlacementBeforeClose = Math.toRadians(40), headingBeacon3PlacementBeforeClose = Math.toRadians(20);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(-20), headingBeacon2PlacementBeforeFar = Math.toRadians(-40), headingBeacon3PlacementBeforeFar = Math.toRadians(-60);
    //END OF LEVINE ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

     */

}
