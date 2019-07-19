package org.firstinspires.ftc.teamcode.RobotLibs.lib;

import org.firstinspires.ftc.teamcode.RobotLibs.lib.hardware.MotorEx;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class PSStorage {

    public HashMap<PSEnum.MotorLoc, MotorEx> driveMotors = new HashMap<PSEnum.MotorLoc, MotorEx>();

    public HashMap<String, MotorEx> motors = new HashMap<String, MotorEx>();

    public void insert(MotorEx motor){
        switch (motor.motorLoc) {
            case RIGHT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case LEFT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case RIGHTFRONT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case LEFTFRONT:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case RIGHTBACK:
                driveMotors.put(motor.motorLoc, motor);
                break;
            case LEFTBACK:
                driveMotors.put(motor.motorLoc, motor);
                break;
        }
        motors.put(motor.motorName, motor);
    }

    public ArrayList<MotorEx> getDrivemotors(PSEnum.MotorLoc motorLoc){
        ArrayList<MotorEx> returnMotors = new ArrayList<MotorEx>();
        for (Map.Entry<PSEnum.MotorLoc, MotorEx> entry : driveMotors.entrySet()) {
            PSEnum.MotorLoc loc = entry.getKey();
            MotorEx motor = entry.getValue();
            if(loc == motorLoc){
                returnMotors.add(motor);
            }


        }
        return returnMotors;
    }
    public ArrayList<MotorEx> getDrivemotors(){
        ArrayList<MotorEx> returnMotors = new ArrayList<MotorEx>();
        for (Map.Entry<PSEnum.MotorLoc, MotorEx> entry : driveMotors.entrySet()) {
            PSEnum.MotorLoc loc = entry.getKey();
            MotorEx motor = entry.getValue();
            if(loc != PSEnum.MotorLoc.NONE){
                returnMotors.add(motor);
            }


        }
        return returnMotors;
    }
}
