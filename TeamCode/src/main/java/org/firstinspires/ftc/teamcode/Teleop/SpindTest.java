package org.firstinspires.ftc.teamcode.Teleop;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanics.Color;
import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Spind;
import org.firstinspires.ftc.teamcode.Mechanics.Turret;
import org.firstinspires.ftc.teamcode.Mechanics.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Spindexer Test")
public class SpindTest extends OpMode {
    private double slot;
    Vision camera = new Vision();
    @Override
    public void loop() {
        if (gamepad1.a) {
            slot = 2;
        }
        if(gamepad1.b){
            try {
                Spind.Launch3Balls("PPG");
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        if (gamepad1.x) {
            Spind.updateBallList();
        }
        //Spind.spinTheDexer(slot);
        for(int i =0;i<3;i++) {
            if(Spind.ballList[i]== Color.DetectedColor.PURPLE)
                telemetry.addData("list " + i, "purple");
            else if(Spind.ballList[i]== Color.DetectedColor.GREEN)
                telemetry.addData("list " + i, "green");
            else if(Spind.ballList[i]== Color.DetectedColor.UNKNOWN)
                telemetry.addData("list " + i, "no ball");
        }
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        Robot.init(hardwareMap);
        camera.initAprilTag(hardwareMap);
        try {
            camera.setManualExposure(6, 250, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        camera.findMotif();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {

    }
}
