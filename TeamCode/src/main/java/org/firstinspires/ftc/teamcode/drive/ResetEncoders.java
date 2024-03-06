package org.firstinspires.ftc.teamcode.drive;

//luh Teleop
//default position pos = 0.9, vpos = 1

/*flat positions:
pos = 0 (70 deg), vpos = 0.205, slidePos = 0.299
pos = 0.05 (72 deg), vpos = 0.21, slidePos = 0.421
pos = 0.1 (75 deg), vpos = 0.214, slidePos = 0.573
*/

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.JayMap;
import org.firstinspires.ftc.teamcode.util.AprilTagAutoAlignmentManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config

public class ResetEncoders extends OpMode {

    JayMap jayBot = new JayMap(this);


    AprilTagAutoAlignmentManager hPosCheck = new AprilTagAutoAlignmentManager();


    @Override
    public void init() {
        jayBot.init();
        jayBot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        terminateOpModeNow();
    }

}