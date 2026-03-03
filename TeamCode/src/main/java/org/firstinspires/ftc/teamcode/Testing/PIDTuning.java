package org.firstinspires.ftc.teamcode.Testing;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanics.Robot;

@Config
@TeleOp(name = "pidtesting", group = "testing")
public class PIDTuning extends OpMode {
    public static double spindPosP,spindPosI,spindPosD,spindPosF,spindVelP,spindVelI,spindVelD,spindVelF,turretP,turretI,turretD,turretF,spindPosTarget,spindVelTarget,turretTarget;
    //working values spindpos{},spindVel{},turret{}
    private PIDFController spindPosController, spindVelController,turretController;
    private PIDFCoefficients spindPosCoef, spindVelCoef,turretCoef;
    boolean spindVel = false;
    boolean aPressed;
    DcMotorEx turret,spindexer;
    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turn");
        spindexer = hardwareMap.get(DcMotorEx.class, "spind");
        spindPosCoef = new PIDFCoefficients(spindPosP,spindPosI,spindPosD,spindPosF);
        spindVelCoef= new PIDFCoefficients(spindVelP,spindVelI,spindVelD,spindVelF);
        turretCoef = new PIDFCoefficients(turretP,turretI,turretD,turretF);
        spindPosController = new PIDFController(spindPosCoef);
        spindVelController = new PIDFController(spindVelCoef);
        turretController = new PIDFController(turretCoef);

    }

    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override

    public void loop() {
        spindPosController.setTargetPosition(spindPosTarget);
        spindVelController.setTargetPosition(spindVelTarget);
        turretController.setTargetPosition(turretTarget);
        spindPosCoef.setCoefficients(spindPosP,spindPosI,spindPosD,spindPosF);
        spindVelCoef.setCoefficients(spindVelP,spindVelI,spindVelD,spindVelF);
        turretCoef.setCoefficients(turretP,turretI,turretD,turretF);
        spindPosController.setCoefficients(spindPosCoef);
        spindVelController.setCoefficients(spindVelCoef);
        turretController.setCoefficients(turretCoef);

        if(spindVel) {
            spindexer.setPower(spindVelController.run());
            spindPosController.updatePosition(spindexer.getVelocity(AngleUnit.RADIANS));
        }else{
            spindexer.setPower(spindPosController.run());
            spindPosController.updatePosition(spindexer.getCurrentPosition());
        }
        turret.setPower(turretController.run());
        turretController.updatePosition(turret.getCurrentPosition());
        if(gamepad1.a&&!aPressed){
            spindVel=!spindVel;
            aPressed=true;
        }else if(!gamepad1.a){
            aPressed=false;
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
