package org.firstinspires.ftc.teamcode.Mechanics;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "LinearSlide", group = "testing")
public class LinearSlide extends OpMode{
    public static double P,I,D,F, targetHeight;
    private double maxheight= Constants.CPR312*3;

    private PIDFController posController;
    private PIDFCoefficients coeff;
    DcMotorEx linearSlide;
    @Override
    public void init() {
        linearSlide = hardwareMap.get(DcMotorEx.class,"ls");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //linearSlide.setDirection(DcMotorSimple.Direction.REVERSE); //uncomment if needed
        coeff = new PIDFCoefficients(P,I,D,F);
        posController = new PIDFController(coeff);
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
    @Override

    public void loop() {
        posController.setTargetPosition(targetHeight);
        coeff.setCoefficients(P,I,D,F);
        posController.setCoefficients(coeff);

        linearSlide.setPower(posController.run());
        posController.updatePosition(linearSlide.getCurrentPosition());
        telemetry.addData("slidePos",linearSlide.getCurrentPosition());
        if(-gamepad1.left_stick_y>0.5){
            targetHeight+=10;
        }else if(-gamepad1.left_stick_y<-0.5){
            targetHeight-=10;
        }
        if(targetHeight>maxheight){
            targetHeight=maxheight;
        }else if(targetHeight<0){
            targetHeight=0;
        }
    }
    @Override
    public void stop() {

    }
}
