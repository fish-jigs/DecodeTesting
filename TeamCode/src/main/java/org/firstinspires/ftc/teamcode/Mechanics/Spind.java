package org.firstinspires.ftc.teamcode.Mechanics;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.intake;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.spindexer;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.transfer;

public class Spind {
    public static double spindexerAngle = 0, targetAngle = 0;
    public static Color.DetectedColor[] ballList = {Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN};
    public static boolean intaking = false;
    public static boolean[] launchedBalls = {false,false,false};
    public static boolean spinTheDexer(double slot) {
        if(intaking)
            targetAngle = Constants.CPR / 3 * (slot+0.5);
        else
            targetAngle = Constants.CPR / 3 * slot;
        spindexer.setTargetPosition((int)Math.round(targetAngle));
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(1);

        spindexerAngle = spindexer.getCurrentPosition();

        if (Math.abs(spindexerAngle - targetAngle) <= 10) {
            spindexer.setPower(0);
            return true;
        }
        return false;
    }

    private static boolean setSpindToColor(Color.DetectedColor color) {
        int index=-1;
        for(int i=0;i<3;i++){
            if (ballList[i]==color) {
                index = i;
                break;
            }
        }
        if(index==-1)
            return true;
        return spinTheDexer(index);
    }

    public static boolean intaking(Timer timer,double timeBetweenSpins){
        if(timer.getElapsedTimeSeconds()>3*timeBetweenSpins){
            intake.setPower(0);
            return true;
        }
        intake.setPower(1);
        intaking=true;
        int index = (int)(timer.getElapsedTimeSeconds()/timeBetweenSpins);
        spinTheDexer(index);
        return false;
    }
    public static boolean updateBallList(Timer timer,double timeBetweenSpins) {
        if(timer.getElapsedTimeSeconds()>3*timeBetweenSpins){
            return true;
        }
        intaking=false;
        int index=(int)(timer.getElapsedTimeSeconds()/timeBetweenSpins);
        if(spinTheDexer(index)&&Color.getColor()!= Color.DetectedColor.UNKNOWN) {
            ballList[index] = Color.getColor();
            timer.resetTimer();
        }
        return false;
    }
    public static int getSigmaPosition() {
        return (int)Math.round(spindexerAngle / Constants.CPR * 3);
    }
    public static boolean Launch3Balls(Timer timer,String motif,double timeBetweenShots) throws InterruptedException {
        if(timer.getElapsedTimeSeconds()>(3*timeBetweenShots)) {
            for(int i =0;i<3;i++)
                launchedBalls[i]=false;
            return true;
        }
        intaking=false;
        int index = (int)(timer.getElapsedTimeSeconds()/timeBetweenShots);
        String[] motifList = motif.split("");
        if(motifList[index].equals("P")){
            if(setSpindToColor(Color.DetectedColor.PURPLE)) {
                launchedBalls[index]=true;
                transfer.setPosition(0.9);
                Thread.sleep(100);
                transfer.setPosition(0.4);
            }
        }
        else if(motifList[index].equals("G")){
            if(setSpindToColor(Color.DetectedColor.GREEN)) {
                launchedBalls[index]=true;
                transfer.setPosition(0.9);
                Thread.sleep(100);
                transfer.setPosition(0.4);
            }
        }
        for(int i =0;i<3;i++){
            if(!launchedBalls[i]){
                spinTheDexer(i);
                transfer.setPosition(0.9);
                Thread.sleep(100);
                transfer.setPosition(0.4);
            }
        }
        return false;
    }
}

