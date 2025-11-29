package org.firstinspires.ftc.teamcode.Mechanics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.spindexer;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.colorSen;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.transfer;

public class Spind {
    public double spindexerAngle;
    private Color colorSen;
    public Color.DetectedColor[] ballList = {Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN};


    private boolean spinTheDexer(int amount) {
        spindexerAngle += Constants.CPR / 6 * amount;
        spindexer.setTargetPosition((int) Math.round(spindexerAngle));
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(1);
        if(spindexer.isBusy())
            return false;
        spindexer.setPower(0);
        return true;
    }

    private boolean setSpindToColor(Color.DetectedColor color) {
        int index=-1;
        for(int i=0;i<3;i++){
            if (ballList[i]==color) {
                index = i;
                break;
            }
        }
        if(index==-1)
            return false;
        return spinTheDexer(Math.min((getSigmaPosition()-index)*2,(3-index)*2));
    }

    private void updateBallList() {
        for(int i =0;i<3;i++){
            ballList[i]= Color.DetectedColor.UNKNOWN;
        }
        int counter2 = 0;
        int counter = 0;
        int time = 100;
        while (counter2 < 6) {
            if(ballList[getSigmaPosition()]== Color.DetectedColor.UNKNOWN)
                ballList[getSigmaPosition()]= colorSen.getColor();
            if (counter > time) {
                spinTheDexer(1);
                counter2++;
                counter = 0;
            }
            counter++;
        }
    }

    public int getSigmaPosition() {
        return (int) ((spindexer.getCurrentPosition()%Constants.CPR) / (Constants.CPR/3))-1;
    }
    private boolean launching = false;
    public boolean Launch3Balls(String motif) throws InterruptedException {
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

