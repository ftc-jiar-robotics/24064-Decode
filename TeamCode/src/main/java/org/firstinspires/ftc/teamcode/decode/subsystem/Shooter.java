package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    final Hood hood;
    final Flywheel flywheel;
    final Turret turret;
    final Feeder feeder;
    final Follower dt;
    final KinematicsSolver kinematicsSolver;

    private boolean
            didCurrentDrop,
            kinematicsValidFullSolve,
            kinematicsValidFixedV;

    private int queuedShots = 0;

    public enum ShooterStates {
        IDLE, PREPPING, RUNNING
    }

    private double launchAngle, turretOffset, idealVLaunch;

    private ShooterStates targetState = ShooterStates.IDLE;

    public Shooter(HardwareMap hw, Follower dt, VoltageSensor voltageSensor) {
        this.hood = new Hood(hw);
        this.flywheel = new Flywheel(hw);
        this.dt = dt;
        this.turret = new Turret(hw, dt, voltageSensor, this);
        this.feeder = new Feeder(hw);
        this.kinematicsSolver = new KinematicsSolver();
    }

    @Override
    public void set(ShooterStates t) {
        targetState = t;
    }

    @Override
    public ShooterStates get() {
        return targetState;
    }

    @Override
    public void setLocked(boolean isLocked) {
        super.setLocked(isLocked);
        feeder.setLocked(isLocked);
        flywheel.setLocked(isLocked);
        turret.setLocked(isLocked);
        hood.setLocked(isLocked);
    }

    public int getQueuedShots() {
        return queuedShots;
    }

    public void incrementQueuedShots(int i) {
        this.queuedShots += i;
    }

    public void setQueuedShots(int i) {
        this.queuedShots = i;
    }

    public boolean isBallPresent() {
        return feeder.isBallPresent();
    }

    public void clearQueueShots() {
        queuedShots = 0;
        targetState = ShooterStates.IDLE;
        turret.set(Turret.TurretStates.IDLE);
        flywheel.set(Flywheel.FlyWheelStates.IDLE, true);
        feeder.set(Feeder.FeederStates.RUNNING, true);
    }

    public void setGoalAlliance() {
        turret.setGoalAlliance();
    }

    public void setFeederIdle(boolean isIdle) {
        if (isIdle) feeder.set(Feeder.FeederStates.RUNNING, false);
    }

    /**
     * @return ideal flywheel velocity (0), compensated hood angle (1), compensated turret angle (2)
     */
    public double[] getCompensatedValues() {
        return new double[]{idealVLaunch, launchAngle, turretOffset};
    }

    public void setTurretManual(Turret.TurretStates t) {
        turret.set(t, true);
    }

    public void applyOffsets() {
        turret.applyOffset();
    }

    public void setFlywheelManual(Flywheel.FlyWheelStates f) {
        flywheel.set(f, true);
    }

    public void setHoodManual(double angleIncrement, boolean isIncrementing) {
        hood.set(hood.get() + (isIncrementing ? angleIncrement : -angleIncrement));
    }

    public void changeFlywheelRPM(double rpmChange, boolean isIncrementing) {
        flywheel.changeRPM(rpmChange, isIncrementing);
    }

    public void changeFlywheelPower(double power) {
        flywheel.setManualPower(power, true);
    }

    @Override
    public void run() {
        kinematicsSolver.setAlliance(isRed);
        kinematicsSolver.setRobotState(dt.getPose(), dt.getVelocity(), dt.getAngularVelocity());

        didCurrentDrop = feeder.didShotOccur();
        if (targetState == ShooterStates.RUNNING && didCurrentDrop) {
            queuedShots = 0;
        }

        kinematicsValidFullSolve = kinematicsSolver.calculateTarget_v_θ_α();
        idealVLaunch = kinematicsSolver.v_launch;

        kinematicsValidFixedV = kinematicsSolver.calculateTarget_θ_α(Flywheel.RPMToInchesPerSecond(flywheel.getCurrentRPMSmooth()));
        launchAngle = kinematicsSolver.θ_launch;
        turretOffset = kinematicsSolver.α_launch;

        switch (targetState) {
            case IDLE:
                feeder.set(Feeder.FeederStates.BLOCKING, true);

                if (!isHoodManual) hood.set(hood.launchRadiansToServoAngle(launchAngle));

                if (queuedShots >= 1) {
                    if (flywheel.get() == Flywheel.FlyWheelStates.IDLE) flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
                    targetState = ShooterStates.PREPPING;
                    if (turret.get() == Turret.TurretStates.IDLE) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case PREPPING:
                if (!isHoodManual) hood.set(hood.launchRadiansToServoAngle(launchAngle));

                if (queuedShots >= 1 && flywheel.get() == Flywheel.FlyWheelStates.RUNNING && turret.isPIDInTolerance() && turret.getDistance() > Common.MIN_SHOOTING_DISTANCE) {
                    feeder.set(Feeder.FeederStates.RUNNING, true);
                    targetState = ShooterStates.RUNNING;
                    if (turret.get() == Turret.TurretStates.IDLE) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case RUNNING:
                if (!isHoodManual) hood.set(hood.launchRadiansToServoAngle(launchAngle));

                flywheel.set(Flywheel.FlyWheelStates.RUNNING, true);
                feeder.set(Feeder.FeederStates.RUNNING, true);

                if (didCurrentDrop) {
                    if (queuedShots <= 0) {
                        targetState = ShooterStates.IDLE;
                        turret.set(Turret.TurretStates.IDLE);
                        flywheel.set(Flywheel.FlyWheelStates.IDLE, true);
                        feeder.set(Feeder.FeederStates.BLOCKING, true);
                    }
                    else {
                        targetState = ShooterStates.RUNNING;
                        feeder.set(Feeder.FeederStates.RUNNING, true);
                    }

                    if (turret.get() == Turret.TurretStates.IDLE) turret.set(Turret.TurretStates.ODOM_TRACKING, true);

                }

                break;
        }

        turret.run();
        flywheel.run();
        feeder.run();
        hood.run();
    }

    @SuppressLint("DefaultLocale")
    public void printTelemetry() {
            turret.printTelemetry();
            flywheel.printTelemetry();
            feeder.printTelemetry();
            hood.printTelemetry();

        telemetry.addLine("SHOOTER");
        telemetry.addData("shooter state (ENUM):", targetState);
        telemetry.addData("queued shots (DOUBLE): ", queuedShots);
        telemetry.addData("did current drop? (BOOLEAN): ", didCurrentDrop);

        telemetry.addLine("KINEMATICS");
        kinematicsSolver.printTelemetry();
        telemetry.addData("full solve valid (BOOLEAN): ", kinematicsValidFullSolve);
        telemetry.addData("fixed-v solve valid (BOOLEAN): ", kinematicsValidFixedV);
    }
}
