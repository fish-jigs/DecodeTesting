package org.firstinspires.ftc.teamcode.Mechanics;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.flywheel;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.hood;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private static double velocity;


    //manual
    public static double getVel() {
        velocity = -flywheel.getVelocity(AngleUnit.RADIANS);
        return velocity;
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
        velocity = -flywheel.getVelocity(AngleUnit.RADIANS);
        double dist = Math.sqrt(x * x + y * y);
        double hoodP = hoodVal(dist, velocity);
        hood.setPosition(hoodP);
        return hoodP;
    }

    private static double hoodVal(double dist, double velocity) {
        double ret = 0;
        if (dist <= 60)
            return 1;
        else if (dist <=81.3) {
            double slope = -0.02662 * dist + 2.4722;
            double y_intercept = .099844 * dist - 9.12812;
            ret = velocity * slope + y_intercept;
            if (ret < 0)
                return 0;
            return ret;
        }
        else if (dist <= 102.12) {
            double slope = -.00623055 * dist + .814544;
            double y_intercept = .021861 * dist - 2.78813;
            ret = velocity * slope + y_intercept;
            if (ret < 0)
                return 0;
            return ret;
        }
        else {
            double slope = -.000200639 * dist + .198769;
            double y_intercept = -.0385703 * dist + 3.38311;
            ret = velocity * slope + y_intercept;
            if (ret < 0)
                return 0;
            return ret;
        }
    }
}
