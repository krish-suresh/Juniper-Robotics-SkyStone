package org.firstinspires.ftc.teamcode.SkyStone.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotLibsV2.Subsystem.Subsystem;
import org.opencv.core.Range;

public class TelemetryDisplay implements Subsystem {
    public OpMode opMode;
    public Telemetry telemetry;
    public int verticalPadding = 1;
    public int horizontalPadding = 1;
    //    char[][] display = new char[64][10];
    char[][] blankFoundation = {
            {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'},
            {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'},
            {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'},
            {'☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐', ' ', '☐'}
    }; //4x16
    /*
     * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
     * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
     * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
     * ☐ ☐ ☐ ☐ ☐ ☐ ☐ ☐
     *
     *
     * */
    char[][] foundation = blankFoundation.clone();
    int[] blockPos = {0, 0};
    boolean isBlockVertical = false;
    public static char filledSquare = '■';
    public static char emptySquare = '☐';

    public TelemetryDisplay(OpMode mode) {
        opMode = mode;
        telemetry = mode.telemetry;
    }

    public void updateDisplay() {
        //put foundation into display
        if (opMode.gamepad2.right_bumper) {
            isBlockVertical = !isBlockVertical;
        }
        blockPos[0] += (opMode.gamepad2.dpad_right ? 1 : (opMode.gamepad2.dpad_left ? -1 : 0));
        blockPos[1] += (opMode.gamepad2.dpad_up ? -1 : (opMode.gamepad2.dpad_down ? 1 : 0));
        com.qualcomm.robotcore.util.Range.clip(blockPos[0], 0, (isBlockVertical ? 7 : 6));
        com.qualcomm.robotcore.util.Range.clip(blockPos[1], 0, (isBlockVertical ? 2 : 3));
        foundation = addBlockPos(blockPos[0], blockPos[1], isBlockVertical);
        for (char[] line : foundation) {
            telemetry.addLine(new String(line));
        }
        telemetry.update();
    }


    /**
     * @param x          xPos from top left corner to left/top peg of stone 0-7
     * @param y          yPos from top left corner to left/top peg of stone 0-3
     * @param isVertical
     */
    public char[][] addBlockPos(int x, int y, boolean isVertical) {
        char[][] foundation = blankFoundation.clone();
        foundation[y][x * 2] = filledSquare;
        if (isVertical) {
            if (y != 3) {
                y++;
            }
        } else {
            x++;
        }
        foundation[y][x * 2] = filledSquare;
        return foundation;
    }

    @Override
    public void update() {
        updateDisplay();
    }
}
