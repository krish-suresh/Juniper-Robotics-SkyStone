package org.firstinspires.ftc.teamcode.SkyStone.r1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Test")
public class test2 extends OpMode {

    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor adi;
    ElapsedTime el = new ElapsedTime();

    @Override
    public void init() {
        leftBackMotor = hardwareMap.dcMotor.get("LB");
        leftFrontMotor = hardwareMap.dcMotor.get("LF");
        rightFrontMotor = hardwareMap.dcMotor.get("RF");
        adi = hardwareMap.dcMotor.get("pineapple");
        adi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        el.seconds();
    }

    @Override
    public void loop() {
        adi.setPower(gamepad1.right_stick_y);
        telemetry.addData("Lift Power", gamepad1.right_stick_y);
    }

}
