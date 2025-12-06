package org.firstinspires.ftc.teamcode.Mechanics;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public static DcMotorEx turret, spindexer, flywheel, intake;

    public static Pose autoEnd = new Pose(72, 72, 0);

    public static Servo hood, transfer;
    public static Vision camera;
    public static void init(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turn");
        spindexer = hardwareMap.get(DcMotorEx.class, "spind");
        flywheel = hardwareMap.get(DcMotorEx.class, "shot");
        intake = hardwareMap.get(DcMotorEx.class, "inta");


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hood = hardwareMap.get(Servo.class, "hood");
        transfer = hardwareMap.get(Servo.class, "trans");

        //camera.initAprilTag(hardwareMap);
        Color.init(hardwareMap);
    }

    public static void reset() {

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
