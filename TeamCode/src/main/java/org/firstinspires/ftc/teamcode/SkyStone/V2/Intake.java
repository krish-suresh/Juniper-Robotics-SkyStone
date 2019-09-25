package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;

public class Intake implements Subsystem {
    public DcMotorEx intakeMotorRight;
    public DcMotorEx intakeMotorLeft;
    public double intakePower;
    //Motors from robot orientation
    public OpMode opMode;

    public Intake(OpMode mode) {
        opMode = mode;
        intakeMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "LI");
        intakeMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "RI");
    }

    @Override
    public void update() {
        intakePower = opMode.gamepad1.right_trigger-opMode.gamepad1.left_trigger; // triggers activate lift
        updateIntakePower();//updates the intake motor powers to the intakePower
    }
    private void updateIntakePower(){
        intakeMotorRight.setPower(-intakePower);
        intakeMotorLeft.setPower(intakePower);
    }
}
