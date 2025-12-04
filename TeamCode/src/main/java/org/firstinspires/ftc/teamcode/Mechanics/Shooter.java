package org.firstinspires.ftc.teamcode.Mechanics;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.flywheel;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.hood;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private static double velocity;


    //manual
    public static double getVel() {
        return velocity;
    }

    public static void setVelocity(double vel) {
        flywheel.setVelocity(vel / 60 * 360, AngleUnit.DEGREES);
    }

    public static void setPower(double power) {
        flywheel.setPower(power);
    }

    public static void setHood(double pos) {
        hood.setPosition(pos);
    }

    public static double getHood() {
        return hood.getPosition();
    }


    //Automation
    public static double autoShotHood(double x, double y) {
        velocity = flywheel.getVelocity();
        double dist = Math.sqrt(x * x + y * y);
        double hoodP = hoodVal(dist, velocity);
        hood.setPosition(hoodP);
        return hoodP;
    }

    private static double hoodVal(double dist, double velocity) {
        return 0;
    }
}
