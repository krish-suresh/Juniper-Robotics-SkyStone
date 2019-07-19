package org.firstinspires.ftc.teamcode.RobotLibs.examples.driveExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSRobot;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.hardware.MotorEx;

/**
 * Created by young on 8/2/2017.
 */


@TeleOp(name = "PineEx-EncoderOneMotor", group = "Linear Opmode")
@Disabled

public class OneMotorEncoderExample extends LinearOpMode {
    PSRobot robot;

    MotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new PSRobot(this);

        motor = robot.motorHandler.newMotor("motor", 1, true , true,40);

        waitForStart();
        //motor.encoderDrive(1, 90, PSEnum.MotorValueType.DEGREES, 4);

        sleep(1000);
        telemetry.addData("Encoder", motor.motorObject.getCurrentPosition());
        telemetry.update();
        sleep(1000);
    }
}


