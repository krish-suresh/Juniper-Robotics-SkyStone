package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryDisplay {
    public OpMode opMode;
    char[][] display = new char[64][20];
    public static char filledSquare = ' ';
    public static char emptySquare = '☐';

    public TelemetryDisplay(OpMode mode){
        opMode = mode;

    }
    public void updateDisplay(){

    }
}
