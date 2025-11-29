package org.firstinspires.ftc.teamcode.Mechanics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public static DcMotorEx turret, spindexer, flywheel, intake;

    public static Servo hood, transfer;
    public static Color colorSen;
    public static void init(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turn");
        spindexer = hardwareMap.get(DcMotorEx.class, "spind");
        flywheel = hardwareMap.get(DcMotorEx.class, "shot");
        intake = hardwareMap.get(DcMotorEx.class, "inta");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        hood = hardwareMap.get(Servo.class, "hood");
        transfer = hardwareMap.get(Servo.class, "trans");


        colorSen.init(hardwareMap);
    }

    public static void reset() {

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
