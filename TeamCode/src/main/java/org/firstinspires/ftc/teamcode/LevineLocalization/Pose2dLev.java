package org.firstinspires.ftc.teamcode.LevineLocalization;

public class Pose2dLev {
    public double xPos;
    public double yPos;
    public double heading;
    public Pose2dLev(double x, double y, double head){
        xPos = x;
        yPos = y;
        heading = Math.toRadians(MathsAndStuff.AngleWrapDeg(head));
    }
    public double getXPos(){
        return xPos;
    }
    public double getYPos(){
        return yPos;
    }
    public double getHeading(){
        return heading;
    }
    public String toString(){
//        return "X: " + xPos + " Y: " + yPos + " Heading: " + Math.toDegrees(heading);
        return "X: " + xPos + " Y: " + yPos + " Heading: " + Math.toDegrees(heading);
    }
}
