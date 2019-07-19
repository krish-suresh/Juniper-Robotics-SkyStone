package org.firstinspires.ftc.teamcode.RobotLibs.lib.auto;

import org.firstinspires.ftc.teamcode.RobotLibs.lib.PSResources;
import org.firstinspires.ftc.teamcode.RobotLibs.lib.drive.Drive;

/**
 * Created by young on 8/6/2017.
 */

public class AutoDrive {

    private PSResources resources;

    private Drive drive;

    /**
     * Constructor of the WorldAuto drive class
     *
     * @param res   Passes the resources for use in methods
     * @param drive gives direct access to the drive methods for ease of use
     */
    public AutoDrive(PSResources res, Drive drive) {
        resources = res;
        this.drive = drive;
    }

}