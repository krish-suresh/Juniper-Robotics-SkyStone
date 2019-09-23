package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class MecanumDrive extends Subsystem {

/*
*    front
* 0         3
*
*
* 1         2
* */


    public OpMode opMode;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx rightFront;
    List<DcMotorEx> driveMotors;
    //TODO add/implement Gyro
    //TODO add/implement ODO modules

    public MecanumDrive(OpMode mode){
        opMode = mode;
        leftFront = opMode.hardwareMap.get(DcMotorEx.class, "");
        leftBack = opMode.hardwareMap.get(DcMotorEx.class, "");
        rightBack = opMode.hardwareMap.get(DcMotorEx.class, "");
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, "");
        driveMotors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : driveMotors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

    @Override
    public void update() {

    }
}
