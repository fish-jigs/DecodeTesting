package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "blueGoalAuto 🥶", group = "testing")
public class AutonomousBlueFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

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
    private final Pose gate2 = new Pose(14.5,72,Math.toRadians(90));
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(startPose,scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading());
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1Pose.getHeading())
                .build();
    }
}