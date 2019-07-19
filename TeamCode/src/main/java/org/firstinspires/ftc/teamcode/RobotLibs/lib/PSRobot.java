package org.firstinspires.ftc.teamcode.RobotLibs.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotLibs.lib.auto.AutoDrive;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.auto.SwitchBoard;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.drive.Drive;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.hardware.handlers.PSMotorHandler;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.hardware.handlers.PSServoHandler;

/**
 * Created by Brandon on 6/26/2017.
 *
 */

public class PSRobot {
    public PSMotorHandler motorHandler;
    public Drive drive;
    public AutoDrive auto;
    private PSResources resources;
    public SwitchBoard switchBoard;
    public PSServoHandler servoHandler;

    public PSRobot(OpMode opMode){
        resources = new PSResources(opMode);
        motorHandler = new PSMotorHandler(resources);
        drive = new Drive(resources);
        auto = new AutoDrive(resources, drive);
        servoHandler = new PSServoHandler(resources);
        switchBoard = new SwitchBoard(resources);
    }

    public void sayFeedBack(String objectName, double value){
        resources.feedBack.sayFeedBackWithOutUpdate(objectName, value);
    }
    public void sayFeedBack(String objectName, String value){
        resources.feedBack.sayFeedBackWithOutUpdate(objectName, value);
    }
    public void updateFeedBack(){
        resources.feedBack.update();
    }

}


