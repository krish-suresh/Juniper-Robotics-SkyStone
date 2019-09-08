package org.firstinspires.ftc.teamcode.SkyStone.V1liftGrab;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSConfigOpMode;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSEnum;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.hardware.MotorEx;

public abstract class Config_r1 extends PSConfigOpMode {

    Drive drive;
    Lift lift;
    @Override
    public void config(OpMode opMode) {
        robot = new PSRobot(opMode);
        drive = new Drive();
        lift = new Lift();
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
            rightBack = robot.motorHandler.newDriveMotor("RB",PSEnum.MotorLoc.RIGHTBACK, 13);
        }

    }
    class Lift {
        MotorEx liftMotor;
        Servo grabber;
        double motorPowerAdding = 0;
        public Lift(){
            liftMotor = robot.motorHandler.newMotor("lift",20);
            liftMotor.motorObject.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            grabber = hardwareMap.servo.get("grab");
        }
        public void liftPower(double power){
            liftMotor.setPower(power+motorPowerAdding);
        }
        public void openGrab(){
            motorPowerAdding = 0.2;
            grabber.setPosition(1);

        }
        public void closeGrab(){
            motorPowerAdding = 0.1;
            grabber.setPosition(0.5);
        }
    }
}
