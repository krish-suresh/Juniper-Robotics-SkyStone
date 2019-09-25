package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;

public class DepositLift implements Subsystem{
    public DcMotorEx liftMotor;
    //TODO add servos
    //TODO auto encoder heights for lift
    //TODO Mag sensor for bottoming out lift
    public OpMode opMode;
    public DepositLift (OpMode mode){
        opMode = mode;
        liftMotor = opMode.hardwareMap.get(DcMotorEx.class,"L");
    }
    @Override
    public void update() {
        liftMotor.setPower(opMode.gamepad2.right_stick_y);
    }


}
