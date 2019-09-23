package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {
    public OpMode opMode;
    public MecanumDrive mecanumDrive;
    public Robot(OpMode mode){
        opMode = mode;
        mecanumDrive = new MecanumDrive(opMode);
    }
}
