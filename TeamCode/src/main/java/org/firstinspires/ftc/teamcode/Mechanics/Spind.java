package org.firstinspires.ftc.teamcode.Mechanics;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.spindexer;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.transfer;

public class Spind {
    public static double spindexerAngle = 0, targetAngle = 0;
    public static Color.DetectedColor[] ballList = {Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN};


    public static boolean spinTheDexer(double slot) {
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

    private static void setSpindToColor(Color.DetectedColor color) {
        int index=-1;
        for(int i=0;i<3;i++){
            if (ballList[i]==color) {
                index = i;
                break;
            }
        }
        if(index!=-1)
            spinTheDexer(index);
    }
    public static void bruh(Telemetry telemetry){
        if(ballList[getSigmaPosition()]== Color.DetectedColor.UNKNOWN)
            ballList[getSigmaPosition()] = Color.getColor(telemetry);
    }
    public static boolean updateBallList(Timer timer,double timeBetweenSpins) {
        if(timer.getElapsedTimeSeconds()>3*timeBetweenSpins){
            return true;
        }
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
    private static boolean launching = false;
    public static boolean Launch3Balls(String motif) throws InterruptedException {
        if (!motif.isEmpty()&&!launching) {
            String[] motifList = motif.split("");
            launching = true;
            for (String a : motifList) {
                if (a.equals("P"))
                    setSpindToColor(Color.DetectedColor.PURPLE);
                else
                    setSpindToColor(Color.DetectedColor.GREEN);
                //launch the balls;
                transfer.setPosition(.9);
                Thread.sleep(75);
                transfer.setPosition(.4);
            }
            launching=false;
        }
        return launching;
    }
}

