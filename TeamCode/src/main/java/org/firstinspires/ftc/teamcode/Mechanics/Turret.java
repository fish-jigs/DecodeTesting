package org.firstinspires.ftc.teamcode.Mechanics;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.turret;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.CPR;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret {
    public static double faceGoal(double x, double y, double theta, boolean red, double offset) {
        //x and y are in inches bc pedropathing is dumb
        double dx = (144) * (red ? 1 : 0) - x;
        double dy = 144 - y;

        double dtheta = Math.atan2(dy,dx) + offset;
        dtheta -= theta;
        dtheta = dtheta % (2 * Math.PI);

        if (dtheta < -Math.toRadians(190)) {
            dtheta += 2 * Math.PI;
        }
        else if (dtheta > Math.PI) {
            dtheta -= 2 * Math.PI;
        }

        int target = (int)Math.round((dtheta / (2 * Math.PI) * 5 * CPR));

        turret.setTargetPosition(target);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);


        return dtheta;
    }
    public static double faceMotif(double x, double y,double theta){
        double dx = 72-x;
        double dy = 144-y;

        double dtheta = Math.atan2(dy,dx);
        dtheta -= theta;
        dtheta = dtheta % (2 * Math.PI);

        if (dtheta < -Math.toRadians(190)) {
            dtheta += 2 * Math.PI;
        }
        else if (dtheta > Math.PI) {
            dtheta -= 2 * Math.PI;
        }
        int target = (int)Math.round((dtheta / (2 * Math.PI) * 5 * CPR));

        turret.setTargetPosition(target);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
        return dtheta;
    }
}
