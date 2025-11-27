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
import org.firstinspires.ftc.teamcode.Mechanics.SigmaClass;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "blueGoalAuto 🥶", group = "testing")
public class AutonomousBlueFar extends OpMode {
    public SigmaClass colorSen= new SigmaClass();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    String motif = "";
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor spind;
    private double spindexerAngle;
    private boolean opModeIsActive = false;
    private int pathState;
    private final Pose startPose = new Pose(60, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(45, 84, Math.toRadians(0));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickupPose1 = new Pose(12, 84, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(45, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickupPose2 = new Pose(12, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 36, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickupPose3 = new Pose(12, 36, Math.toRadians(0));
    private final Pose gate1 = new Pose(48, 72, Math.toRadians(90));
    private final Pose gate2 = new Pose(8.5,72,Math.toRadians(90));
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
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
//
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void spinTheDexer(int amount){
        spindexerAngle+= Constants.CPR/6 *amount;
        spind.setTargetPosition((int)Math.round(spindexerAngle));
        spind.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spind.setPower(1);
    }
    private void setSpindToColor(SigmaClass.DetectedColor color){
        SigmaClass.DetectedColor currentColor = colorSen.getColor();
        int counter2=0;
        int counter=0;
        int time=100;
        while(currentColor!=color&&opModeIsActive&&counter2<6){
            if(counter>time){
                spinTheDexer(1);
                counter2++;
                counter=0;
            }
            currentColor = colorSen.getColor(telemetry);
            telemetry.addData("color:", currentColor.name());
            counter++;
        }
    }
    private void Launch3Balls(){
        if(!motif.isEmpty()){
            String[] motifList= motif.split("");
            for(String a : motifList){
                if(a.equals("P"))
                    setSpindToColor(SigmaClass.DetectedColor.PURPLE);
                else
                    setSpindToColor(SigmaClass.DetectedColor.GREEN);
            }
        }
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy())
                    follower.followPath(scorePreload);
                    setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>3) {
                    follower.followPath(gateSigma, true);
                    setPathState(2);
                }
                break;
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
    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                Thread.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        Thread.sleep(20);
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
        autonomousPathUpdate();

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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        spind=hardwareMap.get(DcMotor.class,"spind");
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        initAprilTag();
        if (USE_WEBCAM) {
            try {
                setManualExposure(6, 250);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //this gets the motif, however it could be inconsistent, so i'll try to make it so the camera points at the thing until it gets the motif and then it can aim towards the goal.
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(AprilTagDetection detection: currentDetections){
            if(motif.isEmpty()) {
                switch (detection.id) {
                    case 21:
                        motif = "GPP";
                        visionPortal.stopStreaming();
                        break;
                    case 22:
                        motif = "PGP";
                        visionPortal.stopStreaming();
                        break;
                    case 23:
                        motif = "PPG";
                        visionPortal.stopStreaming();
                        break;
                    default:
                        telemetry.addData("Skipping", "this sigma is NOT the obelisk", detection.id);
                }

            }
        }
        telemetry.addData("motif",motif);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        colorSen.init(hardwareMap);
        opModeIsActive=true;
        opmodeTimer.resetTimer();
        setPathState(0);

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        opModeIsActive=false;
    }

}