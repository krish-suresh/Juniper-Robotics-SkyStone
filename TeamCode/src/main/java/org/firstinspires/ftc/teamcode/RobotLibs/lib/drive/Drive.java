package org.firstinspires.ftc.teamcode.RobotLibs.lib.drive;

import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSResources;

/**
 * Created by ftcpi on 6/29/2017.
 */

public class Drive extends DriveAbstract {

    public TankDrive tank;
    public MecanumDrive mecanum;

    public Drive(PSResources res) {
        super(res);
        resources = res;
        tank = new TankDrive(resources);
        mecanum = new MecanumDrive(resources);
    }

}