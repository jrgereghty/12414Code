package org.firstinspires.ftc.teamcode.LevineLocalization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.JayMap;

import java.util.ArrayList;

public class ActionRunnerCenterStageAuton {
    public LinearOpMode opMode;
    JayMap jayBot;
    Telemetry telemetry;

    public ActionRunnerCenterStageAuton(LinearOpMode opMode, JayMap jayBot) {
        this.opMode = opMode;
        this.jayBot = jayBot;
        telemetry = new MultipleTelemetry(this.opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void runActions(String action) {
        telemetry.addLine("Run action");
        telemetry.addLine("action is " + action);
        switch (action) {
            case("Close Left Claw"):
                jayBot.closeLeftClaw();
                break;
            case("Slide Reset"):
                jayBot.slideToTarget(0,-1);
                break;
            case("Close Right Claw"):
                jayBot.closeRightClaw();
                break;
            case("Open Left Claw"):
                jayBot.openLeftClaw();
                break;
            case("Open Right Claw"):
                jayBot.openRightClaw();
                break;
            case("Intake Mode"):
                jayBot.setIntakeMode();
                break;
            case("Placing Mode"):
                jayBot.setPlacingMode();
                break;

        }
        telemetry.update();
    }
}
