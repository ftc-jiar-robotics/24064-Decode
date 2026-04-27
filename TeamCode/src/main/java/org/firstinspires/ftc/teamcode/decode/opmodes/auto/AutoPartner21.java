package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.GoalPaths;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "AutoPartner21", preselectTeleOp = "Main TeleOp")

public class AutoPartner21 extends AutoGoal21{

    @Override
    protected void onRun() {
        shootPreload();
        shootSecond();
        shootGateCycle(OFFSETY_CYCLE_ONE, OFFSETX_CYCLE_ONE,1.5, 25.85, false);
        shootGateCycle(OFFSETY_CYCLE_TWO, OFFSETX_CYCLE_TWO,1.5, 25.85, false);
        shootFirst();
        shootGateCycle(OFFSETY_CYCLE_THREE,OFFSETX_CYCLE_THREE,1.5, 25.85, false);
        shootGateCycle(OFFSETY_CYCLE_THREE-.1,OFFSETX_CYCLE_THREE-.1,1.5, 25.85, true);
//        goalLeave();
    }

}