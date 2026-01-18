package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous

public class PedroPathingAuton extends OpMode {

    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState {

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        PATH2_STARTPOS

    }

    PathState pathState;

    private final Pose startPose = new Pose(20.667444574095686, 123.66861143523921, Math.toRadians(145));
    private final Pose shootPose = new Pose(41.38156359393234,84.56009334889146, Math.toRadians(360));
    private final Pose Path2EndPose = new Pose(16.997666277712955, 84.84714119019837, Math.toRadians(0));

    private PathChain path1StartPosEndPos,path1;

    public void buildPath(){

        path1StartPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void stateMachine(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(path1StartPosEndPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done path 1");
                }
                break;
            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }
    //    public void PedroPathingAuton(Follower follower) {
//        Path1 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Pose(20.667, 123.669),
//
//                            new Pose(48.439, 84.056)
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(360))
//
//                .build();
//    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // add limelight stuff here;

        buildPath();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        stateMachine();
    }
}
