package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "V2Tele")
public class Tele extends OpMode {
    Robot robot;
    @Override
    public void init() {
        robot = new Robot(this);
    }

    @Override
    public void loop() {
        robot.update();
    }
}
