package org.firstinspires.ftc.teamcode.SkyStone.r1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "tele_r1", group = "r1")
public class Tele_r1 extends Config_r1 {


    @Override
    public void init() {
        config(this);
    }

    @Override
    public void loop() {
        robot.drive.mecanum.updateMecanum(gamepad1,1);
    }
}
