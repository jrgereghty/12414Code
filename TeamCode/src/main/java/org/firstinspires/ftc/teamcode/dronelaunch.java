package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "planetest")
public class dronelaunch extends OpMode {
    Servo plane;
    @Override
    public void init() {
        plane = hardwareMap.servo.get("plane");
        plane.scaleRange(0.35, 0.51);
        plane.setPosition(0);
    }
    public void loop() {
        if (gamepad1.a){
            plane.setPosition(1);
        }
        if (gamepad1.b){
            plane.setPosition(0);
        }

    }
}
