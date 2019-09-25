package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {
    //This is the robot class where we can create objects for all the subsystems in the robot
    public OpMode opMode;
    //this opmode is the opmode that each tele or auto program extends, it will be passed through the constructor so that we can use gamepads,telemetry, hardwaremap etc.

    //Subsystems, all of them implement the Subsystem interface to insure they have an update method for the robot.update()
    public MecanumDrive mecanumDrive;
    public DepositLift depositLift;
    public Intake intake;
    List<Subsystem> subsystems;

    /**
     * @param mode the opmode from the class who uses the robot to allow this class to have access to gamepads,telemetry, hardwaremap etc.
     */
    public Robot(OpMode mode){
        opMode = mode;
        mecanumDrive = new MecanumDrive(opMode);
        intake = new Intake(opMode);
        subsystems = Arrays.asList(mecanumDrive,intake);//list of subsystems so that we can update all at once
    }

    /**
     * this function updates all the subsystems when called
     */
    public void update(){
        for (Subsystem subsystem:subsystems) {
            subsystem.update();
        }
    }
}
