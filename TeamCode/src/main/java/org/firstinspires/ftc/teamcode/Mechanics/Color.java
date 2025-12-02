package org.firstinspires.ftc.teamcode.Mechanics;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Color {
    static NormalizedColorSensor colorSensor;
    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN,
        NOBALL
    }
    public static void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class,"ColorSensorSigma");
        colorSensor.setGain(20);
    }
    public static DetectedColor getColor(Telemetry telemetry){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed=color.red/color.alpha;
        normGreen=color.green/color.alpha;
        normBlue=color.blue/color.alpha;
        double[] hsv = getHSV(normRed,normGreen,normBlue);
        telemetry.addData("red",normRed);
        telemetry.addData("green",normGreen);
        telemetry.addData("blue",normBlue);
        telemetry.addData("hue", hsv[0]);
        telemetry.addData("saturation",hsv[1]);
        telemetry.addData("value",hsv[2]);
        telemetry.update();
        /*
        purple = >0.54, >0.69, <0.9
        65
        green = <.3, >0.75, <0.8
         */
        if(normRed>0.25&&normGreen<0.5&&normGreen>0.3&&normBlue>0.7)
            return DetectedColor.PURPLE;
        else if(normGreen>0.6&&normRed<0.2&&normBlue>0.5)
            return DetectedColor.GREEN;
        return DetectedColor.UNKNOWN;
    }
    public static DetectedColor getColor(){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed=color.red/color.alpha;
        normGreen=color.green/color.alpha;
        normBlue=color.blue/color.alpha;
        double[] hsv = getHSV(normRed,normGreen,normBlue);
        double hue = hsv[0];
        double sat = hsv[1];
        double value = hsv[2];
        if(value+sat>1.2)
            if(Math.abs(hue-220)<20)
                return DetectedColor.PURPLE;
            else if(Math.abs(hue-167)<20)
                return DetectedColor.GREEN;
        return DetectedColor.UNKNOWN;
    }
    private static double[] getHSV(double r, double g, double b){
        double max = Math.max(r, g);
        max = Math.max(max,b);
        double min = Math.min(r,g);
        min = Math.min(min,b);
        double d = max-min;
        double hue;
        if(max==r){
            hue= 60 * ((g-b)/d%6);
        }
        else if(max==g){
            hue= 60 * ((b-r)/d+2);
        }
        else{
            hue= 60 * ((r-g)/d+4);
        }
        double s;
        if(max==0)
            s=0;
        else
            s=d/max;
        double[] result= {hue,s,max};
        return result;
    }
}