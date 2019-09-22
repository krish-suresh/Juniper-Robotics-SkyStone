package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class V1_TeleOp extends OpMode{

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor leftIntake;
    DcMotor rightIntake;

    Servo leftLift;
    Servo rightLift;
    Servo grabServo;

    double driveSpeed = 1/(1+Math.sqrt(2));         //maximum possible input to 1 motor
    double inputCoeff = 1;                          //in case input wheels are too fast, we can tune them down here

    public void init() {
        leftFront = hardwareMap.dcMotor.get("LF");      //drivetrain motors
        rightFront = hardwareMap.dcMotor.get("RF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightBack = hardwareMap.dcMotor.get("RB");

        leftIntake = hardwareMap.dcMotor.get("LI");     //intake motors
        rightIntake = hardwareMap.dcMotor.get("RI");

        leftLift = hardwareMap.servo.get("LL");         //lift and grab servos
        rightLift = hardwareMap.servo.get("RL");
        grabServo = hardwareMap.servo.get("GS");

        leftFront.setDirection(DcMotor.Direction.REVERSE);      //reverses wheels on left side of drivetrain
        leftBack.setDirection(DcMotor.Direction.REVERSE);       //so wheels will work as expected

        leftIntake.setDirection(DcMotor.Direction.REVERSE);     //same for intake wheels
    }

    public void loop() {
        //takes controller input and sets motors accordingly
        updateDriveTrain();

        //input wheels activated with press of right bumper button
        if(gamepad1.right_bumper) {
            leftIntake.setPower(inputCoeff);
            rightIntake.setPower(inputCoeff);
        }
        //reverse intake with left bumper button
        else if(gamepad1.left_bumper) {
            leftIntake.setPower(-inputCoeff);
            rightIntake.setPower(-inputCoeff);
        }
        //if neither button is pressed, set powers to 0
        else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
        
    }

    public void updateDriveTrain() {
        leftFront.setPower((driveSpeed)*(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x));
        rightFront.setPower((driveSpeed)*(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x));
        leftBack.setPower((driveSpeed)*(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x));
        rightBack.setPower((driveSpeed)*(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x));
    }

}
