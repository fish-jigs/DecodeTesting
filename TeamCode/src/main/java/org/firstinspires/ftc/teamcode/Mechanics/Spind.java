package org.firstinspires.ftc.teamcode.Mechanics;

import com.pedropathing.control.PIDFController;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.flywheel;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.intake;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.sensor1;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.sensor2;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.sensor3;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.spindexer;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.transfer;

public class Spind {
    public static double spindexerAngle = 0, targetAngle = 0;
    public static String[] ballList = {"","",""};
    public static boolean[] launchedBalls = {false,false,false};
    private static boolean p1, p2, p3;
    private static boolean launching = false;
    public static boolean spinTheDexer(double slot) {
        targetAngle = Constants.CPR312 / 3 * slot;
        spindexer.setTargetPosition(spindexer.getCurrentPosition()+(int)targetAngle);
        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setPower(1);

        spindexerAngle = spindexer.getCurrentPosition();

        intake.setPower(-.5);

        if (Math.abs(spindexerAngle - targetAngle) <= 10) {
            spindexer.setPower(0);
            intake.setPower(0);
            return true;
        }
        return false;
    }
    public static boolean intaking(Timer timer, double time){
        if(timer.getElapsedTimeSeconds()>time){
            intake.setPower(0);
            return true;
        }
        intake.setPower(-1);
        return false;
    }

    public static boolean setSpindToMotif(String motif) {
        int index=-1;
        String[] motifList = motif.split("");
        for(int i=0;i<3;i++){
            if (ballList[i].equals(motifList[i])&&ballList[(i+1)%3].equals(motifList[(i+1)%3])&&ballList[(i+2)%3].equals(motifList[(i+2)%3])) {
                index = i;
                break;
            }
        }
        if(index==-1)
            return true;
        return spinTheDexer(index);
    }
    public static void updateBallList() {
        ballList[0] = sensor1.getColor();
        ballList[1] = sensor2.getColor();
        ballList[2] = sensor3.getColor();
    }

    // launches 3 balls
    public static boolean Launch3Balls(String motif){
        if(!setSpindToMotif(motif)) {
            flywheel.setVelocity(4.5, AngleUnit.RADIANS);
            return false;
        }
        return spinTheDexer(-4);
    }
    public static boolean Launch3Balls(Timer timer, double timeBetweenShots,double shooterSpeedTime) throws InterruptedException {
        if (timer.getElapsedTimeSeconds() < .1) {
            p1 = false;
            p2 = false;
            p3 = false;
            System.out.println("please?");
        }
        intake.setPower(-.5);
        if(timer.getElapsedTimeSeconds()<shooterSpeedTime)
            return false;


        System.out.println("Seriously? just here");
        if (!p1 && spinTheDexer(0)) {
            System.out.println("at least we're here");
            transfer.setPosition(.9);
            Thread.sleep(100);
            transfer.setPosition(.4);
            Thread.sleep(75);
            p1 = true;
        }

        if (!p2 && p1 && spinTheDexer(1)) {
            System.out.println("one step closer to freedom");
            transfer.setPosition(.9);
            Thread.sleep(100);
            transfer.setPosition(.4);
            Thread.sleep(75);
            p2 = true;
        }

        if (!p3 && p1 && p2 && spinTheDexer(2)) {
            System.out.println("freedom");
            transfer.setPosition(.9);
            Thread.sleep(100);
            transfer.setPosition(.4);
            Thread.sleep(75);
            p3 = true;
        }

        return p1 && p2 && p3;
    }
}

