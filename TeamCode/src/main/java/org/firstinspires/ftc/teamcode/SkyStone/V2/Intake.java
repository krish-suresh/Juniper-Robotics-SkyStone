package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;

public class Intake extends Subsystem {
    public DcMotorEx intakeMotorRight;
    public DcMotorEx intakeMotorLeft;
    //Motors from robot orientation
    public OpMode opMode;

    public Intake(OpMode mode) {
        opMode = mode;
        intakeMotorLeft = opMode.hardwareMap.get(DcMotorEx.class, "LI");
        intakeMotorRight = opMode.hardwareMap.get(DcMotorEx.class, "RI");
    }

    @Override
    public void update() {
        if (opMode.gamepad1.right_bumper) {
            intakeMotorRight.setPower(1);
            intakeMotorLeft.setPower(-1);
        } else if (opMode.gamepad1.left_bumper) {
            intakeMotorRight.setPower(-1);
            intakeMotorLeft.setPower(1);
        } else {
            intakeMotorRight.setPower(0);
            intakeMotorLeft.setPower(0);
        }
    }
}
