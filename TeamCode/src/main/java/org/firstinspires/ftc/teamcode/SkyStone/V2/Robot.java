package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {
    public OpMode opMode;
    public MecanumDrive mecanumDrive;
    public DepositLift depositLift;
    public Intake intake;
    List<Subsystem> subsystems;
    public Robot(OpMode mode){
        opMode = mode;
        mecanumDrive = new MecanumDrive(opMode);
        intake = new Intake(opMode);
        subsystems = Arrays.asList(mecanumDrive);
    }
    public void update(){
        for (Subsystem subsystem:subsystems) {
            subsystem.update();
        }
    }
}
