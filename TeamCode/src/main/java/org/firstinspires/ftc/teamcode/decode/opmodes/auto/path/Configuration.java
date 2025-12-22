package org.firstinspires.ftc.teamcode.decode.opmodes.auto.path;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isBigTriangle;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.service.quickaccesswallet.SelectWalletCardRequest;
import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

import java.util.ArrayList;
import java.util.Objects;
import java.util.concurrent.Callable;

public class Configuration {

    private static ConfigPaths paths;

    private static Callable<Boolean> isLeaveTime;

    public Configuration(ConfigPaths paths, Callable<Boolean> isLeaveTime) {
        Configuration.paths = paths;
        Configuration.isLeaveTime = isLeaveTime;
    }

    public enum Option {
        RED(() -> new InstantAction(() -> isRed = true)),
        BLUE(() -> new InstantAction(() -> isRed = false)),
        GOAL(() -> new InstantAction(() -> isBigTriangle = true)),
        AUDIENCE(() -> new InstantAction(() -> isBigTriangle = false)),
        PRELOAD(Option::preload),
        INTAKE_HP(Option::intakeHP),
        INTAKE_FIRST(Option::intakeFirst),
        INTAKE_SECOND(Option::intakeSecond),
        INTAKE_THIRD(Option::intakeThird),
        INTAKE_GATE(Option::intakeGate),
        OPEN_GATE(Option::openGate),
        SHOOT(Option::shoot);

        private final Callable<Action> action;

        Option(Callable<Action> action) {
            this.action = action;
        }
        
        public Callable<Action> getAction() {
            return action;
        }

        private static Action preload() {
            paths.preload.getPath(0).setTValueConstraint(0.88);
            return new SequentialAction(
                    new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                    new ParallelAction(
                            new FollowPathAction(robot.drivetrain, paths.preload),
                            RobotActions.shootArtifacts(3, 4, false)
                    ),
                    new InstantAction(() -> Log.d("ConfigAuto", "PRELOAD_FINISH"))
            );
        }
        
        private static Action intakeHP() {
            paths.intakeHP.getPath(0).setTValueConstraint(0.8);
            paths.intakeHPBack.getPath(0).setTValueConstraint(0.775);
            paths.intakeHPRetry.getPath(0).setTValueConstraint(0.88);

            return new SequentialAction(
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_HP_START")),
                    new Actions.TimedAction(
                            new ParallelAction(
                                    new Actions.CallbackAction(new InstantAction(() -> robot.drivetrain.setMaxPower(1)), paths.intakeHP, .01, 0, robot.drivetrain, "INTAKE_HP_SPEED_UP"),
                                    new Actions.CallbackAction(RobotActions.setIntake(1, 0), paths.intakeHP, 0.3, 0, robot.drivetrain, "INTAKE_HP_OBTAIN_BALLS"),
                                    new FollowPathAction(robot.drivetrain, paths.intakeHP, true)
                            ), ConfigPaths.MAX_HP_TIME_MS, "INTAKE_HP_FIRST_TRY"
                    ),
                    new SleepAction(0.3),
                    new Actions.TimedAction(new FollowPathAction(robot.drivetrain, paths.intakeHPBack, true), ConfigPaths.MAX_HP_TIME_MS, "INTAKE_HP_SECOND_TRY_BACK"),
                    new SleepAction(0.3),
                    new Actions.TimedAction(new FollowPathAction(robot.drivetrain, paths.intakeHPRetry, false), ConfigPaths.MAX_HP_TIME_MS, "INTAKE_HP_SECOND_RETRY"),
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_HP_END"))
            );
        }

        private static Action intakeFirst() {
            paths.intakeFirst.getPath(1).setTValueConstraint(0.88);
            paths.intakeFirst.getPath(0).setTValueConstraint(0.88);

            return new SequentialAction(
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_FIRST_START")),
                    new ParallelAction(
                            new Actions.CallbackAction(
                                    new ParallelAction(
                                            new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                                            RobotActions.setIntake(1, 0)
                                    ),
                                    paths.intakeFirst, 0.3, 0, robot.drivetrain, "INTAKE_FIRST_OBTAIN_BALLS"),
                            new FollowPathAction(robot.drivetrain, paths.intakeFirst, true)
                    ),
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_FIRST_END"))
            );
        }

        private static Action intakeSecond() {
            paths.intakeSecond.getPath(1).setTValueConstraint(0.88);
            paths.intakeSecond.getPath(0).setTValueConstraint(0.88);

            return new SequentialAction(
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_SECOND_START")),
                    new ParallelAction(
                            new Actions.CallbackAction(
                                    new ParallelAction(
                                            new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                                            RobotActions.setIntake(1, 0)
                                    ),
                                    paths.intakeSecond, 0.3, 0, robot.drivetrain, "INTAKE_SECOND_OBTAIN_BALLS"),
                            new FollowPathAction(robot.drivetrain, paths.intakeSecond, true)
                    ),
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_SECOND_END"))
            );
        }

        private static Action intakeThird() {
            paths.intakeThird.getPath(1).setTValueConstraint(0.88);
            paths.intakeThird.getPath(0).setTValueConstraint(0.88);

            return new SequentialAction(
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_THIRD_START")),
                    new ParallelAction(
                            new Actions.CallbackAction(
                                    new ParallelAction(
                                            new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                                            RobotActions.setIntake(1, 0)
                                    ),
                                    paths.intakeThird, 0.3, 0, robot.drivetrain, "INTAKE_THIRD_OBTAIN_BALLS"),
                            new FollowPathAction(robot.drivetrain, paths.intakeThird, true)
                    ),
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_THIRD_END"))
            );
        }

        private static Action intakeGate() {
            paths.intakeGate.getPath(0).setTValueConstraint(0.9725);
            paths.intakeGate.getPath(1).setTValueConstraint(0.95);

            return new SequentialAction(
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_GATE_START")),
                    new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                    new ParallelAction(
                            new Actions.CallbackAction(new InstantAction(() -> robot.drivetrain.setMaxPower(0.3)), paths.intakeGate, 0.8, 0, robot.drivetrain, "GATE_INTAKE_SLOW_DOWN"),
                            new Actions.CallbackAction(RobotActions.setIntake(1, 0), paths.intakeGate, 0.01, 1, robot.drivetrain, "GATE_INTAKE_OBTAIN_BALLS"),
                            new FollowPathAction(robot.drivetrain, paths.intakeGate)
                    ),
                    new SleepAction(1.5),
                    new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                    new InstantAction(() -> Log.d("ConfigAuto", "INTAKE_GATE_END"))
            );
        }

        private static Action openGate() {
            paths.openGate.getPath(0).setTValueConstraint(0.9725);

            return new SequentialAction(
                    new InstantAction(() -> Log.d("ConfigAuto", "OPEN_GATE_START")),
                    new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                    new FollowPathAction(robot.drivetrain, paths.openGate),
                    new SleepAction(0.75),
                    new InstantAction(() -> Log.d("ConfigAuto", "OPEN_GATE_END"))
            );
        }

        private static Action shoot() {
            paths.shoot.getPath(0).setTValueConstraint(0.88);

            return new SequentialAction(
                    new InstantAction(() -> Log.d("ConfigAuto", "SHOOT_START")),
                    new InstantAction(() -> robot.drivetrain.setMaxPower(1)),
                    new Actions.UntilConditionAction(
                            isLeaveTime,
                            new SequentialAction(
                                    new ParallelAction(
                                            new Actions.CallbackAction(
                                                    new ParallelAction(
                                                            RobotActions.armTurret(),
                                                            RobotActions.armFlywheel(),
                                                            RobotActions.setIntake(0.25, 0)
                                                    ),
                                                    paths.shoot, 0.01, 0, robot.drivetrain, "SHOOT_ARM_FLYWHEEL_AND_TURRET"
                                            ),
                                            RobotActions.shootArtifacts(3, 4, false),
                                            new FollowPathAction(robot.drivetrain, paths.shoot, true)
                                    )
                            )
                    ),
                    new FollowPathAction(robot.drivetrain, paths.leave),
                    new InstantAction(() -> Log.d("ConfigAuto", "SHOOT_END"))
            );
        }
    }


    private final ArrayList<Option> requirementList = new ArrayList<>();

    private int pointer = -1;

    public final String[] allianceChoices = {
            Option.values()[0].name().replace('_', ' '),
            Option.values()[1].name().replace('_', ' ')
    };
    public final String[] sideChoices = {
            null,
            null,
            Option.values()[2].name().replace('_', ' '),
            Option.values()[3].name().replace('_', ' ')
    };
    public final String[] pathChoices = {
            null,
            null,
            null,
            null,
            Option.values()[4].name().replace('_', ' '),
            Option.values()[5].name().replace('_', ' '),
            Option.values()[6].name().replace('_', ' '),
            Option.values()[7].name().replace('_', ' '),
            Option.values()[8].name().replace('_', ' '),
            Option.values()[9].name().replace('_', ' '),
            Option.values()[10].name().replace('_', ' '),
            Option.values()[11].name().replace('_', ' ')
    };

    public ArrayList<Option> getRequirementList() {
        return requirementList;
    }

    public void doConfig(String[] config, boolean moveCursorUp, boolean moveCursorDown, int screenNumber) {
        if (requirementList.size() > screenNumber) pointer = requirementList.get(screenNumber).ordinal();
        else pointer = -1;

        if (pointer == -1 || config[pointer] == null) while (config[++pointer] == null);


        if (moveCursorUp && pointer + 1 < config.length && config[pointer + 1] != null) {
            pointer++;
        }

        if (moveCursorDown && pointer - 1 >= 0 && config[pointer - 1] != null) {
            pointer--;
        }

        for (int i = 0; i < config.length; i++) {
            if (config[i] != null) dashTelemetry.addLine(i == pointer ? ">> " + config[i] : config[i]);
        }

        if (requirementList.size() > screenNumber) {
            requirementList.set(screenNumber, Option.values()[pointer]);
        } else requirementList.add(Option.values()[pointer]);

        dashTelemetry.addLine("\n==========================================\n");
        dashTelemetry.addLine("Path Combination: " + String.join(" ", requirementList.toString()));
    }

    public void reduceRequireList(int screenNumber) {
        for (int i = requirementList.size() - 1; i > screenNumber; i--) requirementList.remove(i);
    }
}