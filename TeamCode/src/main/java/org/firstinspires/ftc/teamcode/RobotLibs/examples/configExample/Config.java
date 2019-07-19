package org.firstinspires.ftc.teamcode.RobotLibs.examples.configExample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSConfigLinearOpMode;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.hardware.MotorEx;

/**
 * Created by Brandon on 6/26/2017.
 */

abstract public class Config extends PSConfigLinearOpMode {

    public MotorEx testMotor;

    public void config(LinearOpMode linearOpMode){
        robotHandler = new PSRobot(linearOpMode);
        testMotor = robotHandler.motorHandler.newMotor("testMotor", 40);
    }
}
