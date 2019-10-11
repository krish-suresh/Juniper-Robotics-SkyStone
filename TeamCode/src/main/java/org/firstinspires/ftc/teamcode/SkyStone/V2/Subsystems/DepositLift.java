package org.firstinspires.ftc.teamcode.SkyStone.V2.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;
import org.opencv.core.Range;

public class DepositLift implements Subsystem{
    private final double ROTATION_DEFAULT = 0.9;
    private final double ROTATION_ROTATE = 0.35;
    public DcMotorEx liftMotor;
    public Servo grab;
    public Servo rotation;
    public CRServo extention;
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
        extention = opMode.hardwareMap.get(CRServo.class,"D.E");

    }
    @Override
    public void update() {
        liftMotor.setPower((opMode.gamepad2.dpad_up?1:(opMode.gamepad2.dpad_down?-0.4:0)+0.2));
        if (opMode.gamepad2.y){
            grab.setPosition(GRAB_CLOSE);
        } else if(opMode.gamepad2.x) {
            grab.setPosition(GRAB_OPEN);
        }
        extention.setPower(opMode.gamepad2.right_bumper?-0.5:(opMode.gamepad2.left_bumper?0.5:0));

        if (opMode.gamepad2.dpad_right) {
            rotation.setPosition(ROTATION_DEFAULT);
        } else if(opMode.gamepad2.dpad_left) {
            rotation.setPosition(ROTATION_ROTATE);
        }
    }

}
