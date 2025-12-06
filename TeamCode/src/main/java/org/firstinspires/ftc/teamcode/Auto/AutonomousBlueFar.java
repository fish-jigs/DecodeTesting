package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Mechanics.Color;
import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Spind;
import org.firstinspires.ftc.teamcode.Mechanics.Turret;
import org.firstinspires.ftc.teamcode.Mechanics.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "blue far auto 🥶", group = "Blue Testing")
public class AutonomousBlueFar extends OpMode {
    public Color colorSen= new Color();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    String motif = "";
    public Vision camera = new Vision();
    private int pathState;
    private final Pose startPose = new Pose(60, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(45, 84, Math.toRadians(0));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupPose1 = new Pose(16, 84, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(45, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickupPose2 = new Pose(16, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickupPose3 = new Pose(16, 36, Math.toRadians(0));
    private final Pose gate1 = new Pose(48, 72, Math.toRadians(90));
    private final Pose gate2 = new Pose(15,72,Math.toRadians(90));
    private Path scorePreload;
    private PathChain grabPickup1,pickupGrab1, scorePickup1, grabPickup2, pickupGrab2, scorePickup2, grabPickup3,pickupGrab3, scorePickup3,gateSigma,gateSigma2,gateSigma3;
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose,scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading());
        gateSigma = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,gate1))
                .setLinearHeadingInterpolation(scorePose.getHeading(),gate1.getHeading())
                .build();
        gateSigma2 = follower.pathBuilder()
                .addPath(new BezierLine(gate1,gate2))
                .setConstantHeadingInterpolation(gate1.getHeading())
                .build();
        gateSigma3 = follower.pathBuilder()
                .addPath(new BezierLine(gate2,gate1))
                .setConstantHeadingInterpolation(gate1.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gate1,pickup1Pose))
                .setLinearHeadingInterpolation(gate1.getHeading(),pickup1Pose.getHeading())
                .build();
        pickupGrab1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose,pickupPose1))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose1,scorePose))
                .setLinearHeadingInterpolation(pickupPose1.getHeading(),scorePose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup2Pose.getHeading())
                .build();
        pickupGrab2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose,pickupPose2))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2,scorePose))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(),scorePose.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup3Pose.getHeading())
                .build();
        pickupGrab3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose,pickupPose3))
                .setConstantHeadingInterpolation(pickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose3,scorePose))
                .setLinearHeadingInterpolation(pickupPose3.getHeading(),scorePose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()&&Spind.updateBallList(pathTimer,0.75)) {
                    setPathState(14);
                }
                break;
            case 14:
                if(Spind.Launch3Balls(pathTimer, motif, 0.75)){
                    follower.followPath(gateSigma, true);
                    setPathState(2);
                }
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(gateSigma2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(gateSigma3, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pickupGrab1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pickupGrab2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(pickupGrab3,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3,true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

        }
    }
    public void setPathState(int pState){
        pathState=pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        if(motif.isEmpty()){
            motif = camera.findMotif();
        }
        Turret.faceGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), false);
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        Robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        camera.initAprilTag(hardwareMap);
        if (USE_WEBCAM) {
            try {
                camera.setManualExposure(6, 250, telemetry);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //this gets the motif, however it could be inconsistent, so i'll try to make it so the camera points at the thing until it gets the motif and then it can aim towards the goal.
        motif = camera.findMotif();
        telemetry.addData("motif",motif);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

}