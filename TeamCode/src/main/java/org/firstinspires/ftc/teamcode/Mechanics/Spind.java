package org.firstinspires.ftc.teamcode.Mechanics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Spind {
    public DcMotor spindMotor;
    public double spindexerAngle;
    private Color colorSen;
    public Color.DetectedColor[] ballList = {Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN};

    public void init(HardwareMap hardwareMap, Color cs) {
        spindMotor = hardwareMap.get(DcMotor.class, "spind");
        colorSen = cs;
    }

    private void spinTheDexer(int amount) {
        spindexerAngle += Constants.CPR / 6 * amount;
        spindMotor.setTargetPosition((int) Math.round(spindexerAngle));
        spindMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindMotor.setPower(1);
    }

    private void setSpindToColor(Color.DetectedColor color) {
        Color.DetectedColor currentColor = colorSen.getColor();
        int counter2 = 0;
        int counter = 0;
        int time = 100;
        while (currentColor != color && counter2 < 6) {
            if (counter > time) {
                spinTheDexer(1);
                counter2++;
                counter = 0;
            }
            currentColor = colorSen.getColor();
            counter++;
        }
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
        return (int) ((spindMotor.getCurrentPosition()%Constants.CPR) / (Constants.CPR/3));
    }
    private void Launch3Balls(String motif) {
        if (!motif.isEmpty()) {
            String[] motifList = motif.split("");
            for (String a : motifList) {
                if (a.equals("P"))
                    setSpindToColor(Color.DetectedColor.PURPLE);
                else
                    setSpindToColor(Color.DetectedColor.GREEN);
            }
        }
    }
}

