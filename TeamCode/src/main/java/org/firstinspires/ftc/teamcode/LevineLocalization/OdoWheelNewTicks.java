package org.firstinspires.ftc.teamcode.LevineLocalization;

public class OdoWheelNewTicks {
    public double leftOdoWheelTick;
    public double rightOdoWheelTick;
    public double centerOdoWheelTick;

    public OdoWheelNewTicks(double left, double right, double center){
        leftOdoWheelTick = left;
        rightOdoWheelTick = right;
        centerOdoWheelTick = center;
    }
    public String toString(){
        return "LeftWheelNew " + leftOdoWheelTick + " RightWheelNew " + rightOdoWheelTick + " CenterWheelNew " + centerOdoWheelTick;
    }
}
