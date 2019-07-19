package org.firstinspires.ftc.teamcode.SkyStone.r1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.drive.TwoTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Paths.ConstantsLoader;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.hardware.MotorEx;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.apache.commons.math3.util.Precision.EPSILON;

public abstract class Config_r1 extends PSConfigOpMode {

    Drive drive;

    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
        drive = new Drive();
    }

    class Drive {
        MotorEx leftFront;
        MotorEx rightFront;
        MotorEx leftBack;
        MotorEx rightBack;

        public Drive() {
            leftFront = robot.motorHandler.newDriveMotor("LF", PSEnum.MotorLoc.LEFTFRONT, 13);
            rightFront = robot.motorHandler.newDriveMotor("RF", PSEnum.MotorLoc.RIGHTFRONT, 13);
            leftBack = robot.motorHandler.newDriveMotor("LB", PSEnum.MotorLoc.LEFTBACK, 13);
            rightBack = robot.motorHandler.newDriveMotor("RB", PSEnum.MotorLoc.RIGHTBACK, 13);
        }

    }
}
