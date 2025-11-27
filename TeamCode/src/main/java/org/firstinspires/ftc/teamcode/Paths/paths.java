package org.firstinspires.ftc.teamcode.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;

public class paths {
    public static PathChain Preload, Pickup1, Gate, Score1, Pickup2, Score2, Pickup3, Score3;

    public static void blueGoal(Follower follower) {
        Preload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.2, 110),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Pickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(70.000, 90.000),
                                new Pose(24.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(15))
                .build();

        Gate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(24.000, 84.000),
                                new Pose(15.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                .build();

        Score1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(15.000, 72.000),
                                new Pose(40.000, 75.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                .build();

        Pickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(50.000, 63.000),
                                new Pose(24.000, 63.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(15))
                .build();

        Score2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.000, 63.000),
                                new Pose(50.000, 63.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(15))
                .build();

        Pickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 84.000),
                                new Pose(60.000, 36.000),
                                new Pose(24.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(15))
                .build();

        Score3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(24.000, 36.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(15))
                .build();
    }
}
