package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.Paths.paths;

// Pedro imports
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//vision imports
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.Mechanics.Vision;

//FTC imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

@Autonomous(name = "Blue Goal Auto 🥶", group = "Blue Testing")
public class AutoBlueGoal extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Vision camera;
    private String pathState;


    public void autonomousPathUpdate() {
        switch (pathState) {
            case "0":
                if (!follower.isBusy()) {
                    follower.followPath(paths.Preload, true);
                    setPathState("Goal 3");
                }
                break;
            case "Goal 3":

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.Pickup1, true);
                    setPathState("Gate");
                }
                break;
            case "ScoreGoal3":
                if (!follower.isBusy()) {
                    follower.followPath(paths.Score1,true);
                    setPathState("Middle 3");
                }
                break;
            case "Gate":
                if (!follower.isBusy()) {
                    follower.followPath(paths.Gate,true);
                    setPathState("ScoreGoal3");
                }
                break;
            case "Middle 3":
                if (!follower.isBusy()) {
                    follower.followPath(paths.Pickup2,true);
                    setPathState("ScoreMiddle3");
                }
                break;
            case "ScoreMiddle3":
                if (!follower.isBusy()) {
                    follower.followPath(paths.Score2,true);
                    setPathState("Far 3");
                }
                break;
            case "Far 3":
                if (!follower.isBusy()) {
                    follower.followPath(paths.Pickup3,true);
                    setPathState("ScoreFar3");
                }
                break;
            case "ScoreFar3":
                if (!follower.isBusy()) {
                    follower.followPath(paths.Score3,true);
                    setPathState("");
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(String pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        paths.blueGoal(follower);
        follower.setStartingPose(new Pose(15.2, 110));

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        //this gets the motif, however it could be inconsistent, so i'll try to make it so the camera points at the thing until it gets the motif and then it can aim towards the goal.

        telemetry.addData("motif",camera.motif);
        telemetry.update();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
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

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState("0");
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {

    }
}