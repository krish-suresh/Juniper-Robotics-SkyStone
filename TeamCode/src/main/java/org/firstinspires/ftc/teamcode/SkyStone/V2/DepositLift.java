package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;
import org.opencv.core.Range;

public class DepositLift implements Subsystem{
    public DcMotorEx liftMotor;
    public Servo grab;
    public Servo rotation;
    public CRServo extR;
    public CRServo extL;
    //TODO auto encoder heights for lift
    //TODO Mag sensor for bottoming out lift
    public OpMode opMode;
//    private double rPos = 0;
//    private double lPos = 1;
//    public double incVal = 0.005;
    public double extensionPower=0;
    public final double EXTENSION_SPEED=0.5;
    public int liftHeight = 0;
    public final double GRAB_CLOSE=0.2;
    public final double GRAB_OPEN=0;

    public DepositLift (OpMode mode){
        opMode = mode;
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class,"L");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        grab = opMode.hardwareMap.get(Servo.class,"D.G");
        rotation = opMode.hardwareMap.get(Servo.class,"D.R");
        extR = opMode.hardwareMap.get(CRServo.class,"E.R");
        extL = opMode.hardwareMap.get(CRServo.class,"E.L");
    }
    @Override
    public void update() {
        liftMotor.setPower((opMode.gamepad1.dpad_up?1:(opMode.gamepad1.dpad_down?-0.4:0)+0.2));
        if (opMode.gamepad1.a){
            grab.setPosition(GRAB_CLOSE);
        } else if(opMode.gamepad1.x) {
            grab.setPosition(GRAB_OPEN);
        }
        extensionPower = opMode.gamepad1.right_bumper?EXTENSION_SPEED:(opMode.gamepad1.left_bumper?-EXTENSION_SPEED:0);
        extR.setPower(extensionPower);
        extR.setPower(-extensionPower);
//        rPos = com.qualcomm.robotcore.util.Range.clip(rPos+(opMode.gamepad1.left_bumper?incVal:(opMode.gamepad1.right_bumper?-incVal:0)),-1,1);
//        lPos = com.qualcomm.robotcore.util.Range.clip(lPos+(opMode.gamepad1.left_bumper?-incVal:(opMode.gamepad1.right_bumper?incVal:0)),-1,1);
//
//        extR.setPosition(rPos);
//        extL.setPosition(lPos);
    }


}
