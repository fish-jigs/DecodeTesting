package org.firstinspires.ftc.teamcode.Mechanics;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SigmaClass {
    NormalizedColorSensor colorSensor;
    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN
    }
    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class,"ColorSensorSigma");
        colorSensor.setGain(20);
    }
    public DetectedColor getColor(Telemetry telemetry){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed=color.red/color.alpha;
        normGreen=color.green/color.alpha;
        normBlue=color.blue/color.alpha;

        telemetry.addData("red",normRed);
        telemetry.addData("green",normGreen);
        telemetry.addData("blue",normBlue);
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
    public DetectedColor getColor(){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed=color.red/color.alpha;
        normGreen=color.green/color.alpha;
        normBlue=color.blue/color.alpha;

        if(normRed>0.5&&normGreen<0.8&&normGreen>0.57&&normBlue>0.9)
            return DetectedColor.PURPLE;
        else if(normGreen>0.9&&normRed<0.4)
            return DetectedColor.GREEN;
        return DetectedColor.UNKNOWN;
    }
}