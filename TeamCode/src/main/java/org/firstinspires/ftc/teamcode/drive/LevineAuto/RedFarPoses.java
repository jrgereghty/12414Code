package org.firstinspires.ftc.teamcode.drive.LevineAuto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RedFarPoses {
    //Starting pos
    //public static double xPosStartingPos = -31, yPosStartingPos = -58;
    public static double sideMultiplierX = 1;
    public static double sideMultipliery = -1;
    public static double angleMultiplier = -1;
    public static double xPosStartingPos = 36 * sideMultiplierX, yPosStartingPos = 62.5 * sideMultipliery, headingStartingPos = Math.toRadians(90);
    public static double xPosStartingPos2 = 36 * sideMultiplierX, yPosStartingPos2 = 53 * sideMultipliery;

    //Purple Pixel Placement
    //NOT ADJUSTED
    public static double xPosLeftSpikeMark = -20, yPosLeftSpikeMark = 35 * sideMultipliery, headingLeftSpikeMark = Math.toRadians(180);//x,40,y35,y = 180 Left
    //x=44.5,
    public static double xPosMiddleSpikeMark = -12, yPosMiddleSpikeMark = 49 * sideMultipliery, headingMiddleSpikeMark = Math.toRadians(90);
    public static double xPosRightSpikeMark = -40, yPosRightSpikeMark = 35 * sideMultipliery, headingRightSpikeMark = Math.toRadians(180);
    public static double xPosTurn2BoardMid = 12, yPosTurn2BoardMid = 49 * sideMultipliery, headingTurn2BoardMid = Math.toRadians(0);
    //x=20,y35, H = 180 Right
    //x=13 ,y=49, heading = 270 Mid


    //Poses For board Placing
    public static double xPosLeftBoardPlace = 44.5, yPosLeftBoardPlace = 45 * sideMultipliery;
    public static double xPosMidBoardPlace = 44.5, yPosMidBoardPlace = 30.5 * sideMultipliery;
    public static double xPosRightBoardPlace = 44.5, yPosRightBoardPlace = 31.5 * sideMultipliery;
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
    public static double xPosStartParkMid = 9, yPosStartParkMid = 40* sideMultipliery;
    public static double xPosEndParkMid = 9, yPosEndParkMid = 15* sideMultipliery;
    public static double xPosStartParkEdge = 61, yPosStartParkEdge = 40* sideMultipliery;
    public static double xPosEndParkEdge = 61, yPosEndParkEdge = 15 * sideMultipliery;
    public static double faceBoard = Math.toRadians(0), faceWhites = Math.toRadians(180);
    //Old Style purple placement:
    public static double xPosLine4StartLeft = 12, yPosLine4StartLeft = 53 * sideMultipliery, headingLine4StartLeft = Math.toRadians(307);
    public static double xPosLine4StartMid = 12, yPosLine4StartMid = 51 * sideMultipliery, headingLine4StartMid = Math.toRadians(270);
    public static double xPosLine4StartRight = 12, yPosLine4StartRight = 53 * sideMultipliery, headingLine4StartRight = Math.toRadians(245);
    public static double xPosReset4Left = 12, yPosReset4Left = 59 * sideMultipliery;//Heading is face board
    //After only poses


    /*
    public static double xPosPurplePixelPlacementAfterBeacon1 = 30.4, yPosPurplePixelPlacementAfterBeacon1 = -30;
    public static double xPosPurplePixelPlacementAfterBeacon23 = 22.4, yPosPurplePixelPlacementAfterBeacon23 = -30;
    public static double xPosStartExtendFirstPlacementAfter = 40, yPosStartExtendFirstPlacementAfter = -27;
    public static double xPosFirstPlacementAfter = 50, yPosFirstPlacementAfterBeacon1 = -19, yPosFirstPlacementAfterBeacon2 = -27, yPosFirstPlacementAfterBeacon3 = -32;

    //Before only poses
    public static double xPosGoAcrossForBeforeTrussPurplePixelFar = -29, yPosGoAcrossForBeforeTrussPurplePixelFar = -10;
    public static double xPosGoAroundPurplePixelBeacon2 = -50, yPosGoAroundPurplePixelBeacon2 = -15;
    public static double xPosGoAcrossForBeforeTrussPurplePixelClose = -36, yPosGoAcrossForBeforeTrussPurplePixelClose = -45;

    //Far poses
    public static double xPosLineUpForPickUpFar = 40, yPosLineUpForPickUpFar = -5;
    public static double xPosLineUpForPlaceFar = 15, yPosLineUpForPlaceFar = -5;
    public static double xPosStartArmExtendPickUpFar = -21, yPosStartArmExtendPickUpFar = -5;
    public static double xPosPickUpPixelFar = -43, yPosPickUpPixelFar = -5;
    public static double xPosPlacePixelFar = 47, yPosPlacePixelFar = -22;
    public static double xPosFlipAfterPlaceFar = 24, yPosFlipAfterPlaceFar = -16;
    public static double xPosStartArmExtendPlaceFar = 30, yPosStartArmExtendPlaceFar = -20;

    //Close poses
    public static double xPosLineUpForPickUpClose = 13, yPosLineUpForPickUpClose = -54.5;
    public static double xPosStartArmExtendPickUpClose = 0, yPosStartArmExtendPickUpClose = -52;
    public static double xPosPickUpPixelClose = -43, yPosPickUpPixelClose = -44;
    public static double xPosStartArmExtendPlaceClose = 30, yPosStartArmExtendPlaceClose = -54.5;
    public static double xPosPlacePixelClose = 47, yPosPlacePixelClose = -42;
    public static double xPosFlipAfterPlaceClose = 20, yPosFlipAfterPlaceClose = -42;
    public static double xPosGoStraightThroughTrussClose = -40, yPosGoStraightThroughTrussClose = -54.5;
    public static double xPosMidStackPickUpFar = -40, yPosMidStackPickUpFar = -13;
    public static double xPosMidStackPickUpClose = -50, yPosMidStackPickUpClose = -50;

    //Park Poses
    public static double xPosParkTriangle = 53, yPosParkTriangle = -9;
    public static double xPosLineUpParkTriangle = 40, yPosLineUpParkTriangle = -9;
    public static double xPosParkSquare = 53, yPosParkSquare = -55;
    public static double xPosLineUpParkSquare = 40, yPosLineUpParkSquare = -55;

    public static double headingMidStackPickUpFar = Math.toRadians(150), headingMidStackPickUpClose = Math.toRadians(230);
    public static double headingStartingPositionAndBeacon = Math.toRadians(90), headingPickUpClose = Math.toRadians(215);
    public static double headingPlaceFar = Math.toRadians(160), headingPlaceClose = Math.toRadians(200);
    public static double headingWallBeaconBefore = Math.toRadians(69), headingTrussBeaconBefore = Math.toRadians(105), headingTiltedBeaconsAfter = Math.toRadians(60);
    public static double headingMidBeaconBefore = Math.toRadians(80), headingBeacon1PlacementAfter = Math.toRadians(195), headingBeacon3PlacementAfter = Math.toRadians(161);
    public static double headingBeacon1PlacementBeforeClose = Math.toRadians(220), headingBeacon2PlacementBeforeClose = Math.toRadians(200), headingBeacon3PlacementBeforeClose = Math.toRadians(190);
    public static double headingBeacon1PlacementBeforeFar = Math.toRadians(220), headingBeacon2PlacementBeforeFar = Math.toRadians(200), headingBeacon3PlacementBeforeFar = Math.toRadians(190);

     */
}
