package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Paths.paths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "turret Test", group = "test")
public class Test extends OpMode {
    DcMotor turret;
    @Override
    public void loop() {
        telemetry.addData("pos",turret.getCurrentPosition());
    }
    @Override
    public void start() {
        turret = hardwareMap.get(DcMotor.class,"turn");
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {

    }
    @Override
    public void init() {

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

}
