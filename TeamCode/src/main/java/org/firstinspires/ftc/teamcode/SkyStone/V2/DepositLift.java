package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;

public class DepositLift extends Subsystem{
    public DcMotorEx liftMotor;
    public OpMode opMode;
    public DepositLift (OpMode mode){
        opMode = mode;
//        liftMotor = opMode.hardwareMap.get(DcMotorEx.class,"");
    }
    @Override
    public void update() {

    }
}
