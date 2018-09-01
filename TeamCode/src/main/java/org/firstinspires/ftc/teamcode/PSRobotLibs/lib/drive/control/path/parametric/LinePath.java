package org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.path.parametric;

import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Angle;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Pose2d;
import org.firstinspires.ftc.teamcode.PSRobotLibs.lib.drive.control.Vector2d;


public class LinePath implements ParametricPath {
    private Pose2d start, end;
    private double headingDelta;

    public LinePath(Pose2d start, Pose2d end) {
        this.start = start;
        this.end = end;
        headingDelta = Angle.norm(end.heading() - start.heading());
    }

    @Override
    public double length() {
        return Vector2d.distance(start.pos(), end.pos());
    }

    @Override
    public Pose2d start() {
        return start;
    }

    @Override
    public Pose2d end() {
        return end;
    }

    @Override
    public Pose2d getPose(double displacement) {
        double t = displacement / length();
        return new Pose2d(start.pos().added(end.pos().added(start.pos().negated()).multiplied(t)),
                Angle.norm(start.heading() + t * headingDelta));
    }

    @Override
    public Pose2d getDerivative(double displacement) {
        return new Pose2d(end.pos().added(start.pos().negated()), headingDelta).multiplied(1 / length());
    }

    @Override
    public Pose2d getSecondDerivative(double displacement) {
        return new Pose2d(0, 0, 0);
    }
}
