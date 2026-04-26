package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    final Hood hood;
    final Flywheel flywheel;
    final Turret turret;
    final Feeder feeder;
    final KinematicsSolver kinematicsSolver;

    private boolean
            didShotOccur,
            inEmergency,
            kinematicsValidFullSolve,
            kinematicsValidFixedV;

    private int queuedShots = 0;
    private int ballConfidence = 0;

    public static double TRAJ_MULT = 1; // 0.862968 TODO TUNE!!
    private double launchAngle, turretOffset, idealVLaunch;

    public enum ShooterStates {
        IDLE, PREPPING, RUNNING
    }

    private ShooterStates targetState = ShooterStates.IDLE;

    public Shooter(HardwareMap hw, boolean isAuto) {
        this.hood = new Hood(hw);
        this.flywheel = new Flywheel(hw);
        this.turret = new Turret(hw);
        this.kinematicsSolver = new KinematicsSolver();
        this.feeder = new Feeder(hw, isAuto);
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

    public static double
            ALL_BALL_CONFIDENCE_THRESHOLD = 2,
            MIN_SHOOTING_DISTANCE = 40,
            HOOD_DISTANCE_SHOOTER_SWITCH = 1000;

    /**
     * @return ideal flywheel velocity (0), compensated hood angle (1), compensated turret angle (2)
     */
    public double[] getCompensatedValues() {
        return new double[]{idealVLaunch, launchAngle, turretOffset};
    }

    public double rpmDropFromCurrentRPM(double rpm) {
        return rpm; // TODO CURVE FIT RPM DROP AND REPLACE RPM DROP VARIABLE
    }

    public int getQueuedShots() {
        return queuedShots;
    }

    public double getFeederSpeed(){
        return feeder.getSpeed();
    }
//    public double getTurretAngle() {
//        return turret.getCurrentAngle();
//    }

    public void armFlywheel() {
        flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
    }

    public void incrementQueuedShots(int i) {
        this.queuedShots += i;
    }

    public void setQueuedShots(int i) {
        this.queuedShots = i;
    }

    public boolean isBallPresent() {
        return feeder.isBallPresent() || robot.intake.getFrontState() || robot.intake.getBackState();
    }

    public boolean isBallInFeeder() {
        return feeder.isBallPresent();
    }
    public boolean isBallInIntakeFront() {
        return robot.intake.getFrontState();
    }
    public boolean isBallInIntakeBack() {
        return robot.intake.getBackState();
    }

    public boolean isRobotFullWithBalls() {
        return ballConfidence > ALL_BALL_CONFIDENCE_THRESHOLD;
    }

//    public void closeAutoAim() {
//        turret.closeAutoAim();
//    }

    public void clearQueueShots() {
        queuedShots = 0;
        targetState = ShooterStates.IDLE;
        turret.set(Turret.TurretStates.IDLE);
        flywheel.set(Flywheel.FlyWheelStates.IDLE, true);
        feeder.set(Feeder.FeederStates.RUNNING, true);
    }

    public void setGoalAlliance() {
        turret.setAlliance();
        kinematicsSolver.setAlliance(Common.isRed);
    }

    public void setFeederIdle(boolean isIdle) {
        if (isIdle) feeder.set(Feeder.FeederStates.RUNNING, false);
    }

    public void turnOnEmergency() {
        if (robot.shooter.get() == ShooterStates.PREPPING) inEmergency = true;
    }

    public void setTurretManual(Turret.TurretStates t) {
        turret.set(t, true);
    }

    public void applyOffsets() {
//        turret.applyOffset();
    }
    public void setFlywheelManual(Flywheel.FlyWheelStates f) {
        flywheel.set(f, true);
    }

    public void setHoodManual(double angleIncrement, boolean isIncrementing) {
        hood.set(hood.get() + (isIncrementing ? angleIncrement : -angleIncrement));
    }

    public void incrementFlywheelRPM(double rpmIncrement, boolean isIncrementing) {
        flywheel.incrementFlywheelRPM(rpmIncrement, isIncrementing);
    }


    @Override
    public void run() {
        kinematicsSolver.setRobotState(robot.drivetrain.getPose(), robot.drivetrain.getVelocity(), robot.drivetrain.getAngularVelocity());

        if (isBallInFeeder() && isBallInIntakeFront() && isBallInIntakeBack()) {
            ballConfidence++;
        }
        else {
            ballConfidence = 0;
        }
        didShotOccur = feeder.didShotOccur();
        if (targetState == ShooterStates.RUNNING && didShotOccur) {
            queuedShots = 0;
        }

        kinematicsValidFullSolve = kinematicsSolver.calculateTarget_v_θ_α();
        idealVLaunch = kinematicsSolver.v_launch;

        kinematicsValidFixedV = kinematicsSolver.calculateTargetWithVelocity_θ_α(idealVLaunch * TRAJ_MULT, robot.isFar);
        launchAngle = kinematicsSolver.θ_launch;
        turretOffset = kinematicsSolver.α_launch;

        switch (targetState) {
            case IDLE:
                feeder.set(Feeder.FeederStates.BLOCKING, true);
                if (!isHoodManual) hood.set(hood.launchRadiansToServoAngle(launchAngle));

                if (queuedShots >= 1) {
                    if (flywheel.get() == Flywheel.FlyWheelStates.IDLE) flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
                    targetState = ShooterStates.PREPPING;
                    if (turret.get() != Turret.TurretStates.ODOM_TRACKING) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case PREPPING:
                double distance = turret.getDistance();
                if (!isHoodManual) hood.set(hood.launchRadiansToServoAngle(launchAngle));

                if ((queuedShots >= 1 &&
                        flywheel.get() == Flywheel.FlyWheelStates.RUNNING &&
                        turret.isPIDInTolerance() &&
                        (robot.isAuto || distance > MIN_SHOOTING_DISTANCE) &&
                        (distance <= 120 || turret.isReadyToShoot())) || inEmergency) {
                    inEmergency = false;
                    feeder.set(Feeder.FeederStates.RUNNING, true);

                    targetState = ShooterStates.RUNNING;
                    if (turret.get() != Turret.TurretStates.ODOM_TRACKING) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case RUNNING:
                if (!isHoodManual) hood.set(hood.launchRadiansToServoAngle(launchAngle));

                flywheel.set(Flywheel.FlyWheelStates.RUNNING, true);
                feeder.set(Feeder.FeederStates.RUNNING, true);

                if (didShotOccur) {
                    if (queuedShots <= 0) {
                        targetState = ShooterStates.IDLE;
                        turret.set(Turret.TurretStates.IDLE);
                        flywheel.set(Flywheel.FlyWheelStates.IDLE, true);
                        feeder.set(Feeder.FeederStates.BLOCKING, true);
                    } else {
                        targetState = ShooterStates.RUNNING;
                        feeder.set(Feeder.FeederStates.RUNNING, true);
                    }

                    if (turret.get() == Turret.TurretStates.IDLE) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
        }

        turret.run();
        flywheel.ballIsPresent = isBallPresent();
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
        telemetry.addData("did current drop? (BOOLEAN): ", didShotOccur);

        Common.dashTelemetry.addLine("KINEMATICS");
        kinematicsSolver.printTelemetry();
        Common.dashTelemetry.addData("full solve valid (BOOLEAN): ", kinematicsValidFullSolve);
        Common.dashTelemetry.addData("fixed-v solve valid (BOOLEAN): ", kinematicsValidFixedV);
        Common.dashTelemetry.addData("v_launch (IPS): ", kinematicsSolver.v_launch);
        Common.dashTelemetry.addData("v_launch calculated RPM: ", flywheel.inchesPerSecondToRPM(kinematicsSolver.v_launch));
        Common.dashTelemetry.addData("θ_launch ideal (RAD): ", kinematicsSolver.θ_launch);
        Common.dashTelemetry.addData("θ_launch ideal hood angle (DEG): ", hood.launchRadiansToServoAngle(kinematicsSolver.θ_launch));
        Common.dashTelemetry.addData("θ_launch compensated (RAD): ", launchAngle);
        Common.dashTelemetry.addData("θ_launch compensated hood angle (DEG): ", hood.launchRadiansToServoAngle(launchAngle));
        Common.dashTelemetry.addData("α_launch ideal (RAD): ", getCompensatedValues()[2]);
    }
}
