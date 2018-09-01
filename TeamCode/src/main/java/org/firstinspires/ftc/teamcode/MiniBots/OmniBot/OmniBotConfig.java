package org.firstinspires.ftc.teamcode.MiniBots.OmniBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSEnum;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSMotor;
import org.firstinspires.ftc.teamcode.PSRobotPackage.lib.PSRobot;

abstract public class OmniBotConfig extends PSConfigOpMode {

    PSMotor leftFront;
    PSMotor rightFront;
    PSMotor leftBack;
    PSMotor rightBack;
    GyroSensor gyroSensor;
    PSMotor[] psMotors = {leftFront,rightFront,rightBack,leftBack};

    @Override
    public void config(OpMode opMode) {
        robotHandler = new PSRobot(opMode);
        gyroSensor = hardwareMap.gyroSensor.get("GS");
        leftFront = robotHandler.motorHandler.newDriveMotor("LF", PSEnum.MotorLoc.LEFTFRONT);
        rightFront = robotHandler.motorHandler.newDriveMotor("RF", PSEnum.MotorLoc.RIGHTFRONT);
        leftBack = robotHandler.motorHandler.newDriveMotor("LB", PSEnum.MotorLoc.LEFTBACK);
        rightBack = robotHandler.motorHandler.newDriveMotor("RB", PSEnum.MotorLoc.RIGHTBACK);

    }
}
