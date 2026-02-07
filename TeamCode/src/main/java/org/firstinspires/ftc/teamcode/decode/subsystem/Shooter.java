package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.ANG_VELOCITY_MULTIPLER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.BLUE_GOAL_ID;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import static java.lang.Math.atan2;
import static java.lang.Math.round;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Shooter extends Subsystem<Shooter.ShooterStates> {
    final Hood hood;
    final Flywheel flywheel;
    final Turret turret;
    final Feeder feeder;

    private boolean
            didShotOccur,
            inEmergency;

    private int queuedShots = 0;
    private int ballConfidence = 0;
    public enum ShooterStates {
        IDLE, PREPPING, RUNNING
    }

    public static double SCORE_HEIGHT = 26; //inches
    public static double SCORE_ANGLE = Math.toRadians(-30); //radians
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    private ShooterStates targetState = ShooterStates.IDLE;

    public Shooter(HardwareMap hw) {
        this.hood = new Hood(hw);
        this.flywheel = new Flywheel(hw);
        this.turret = new Turret(hw);
        this.feeder = new Feeder(hw);
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

    public static double HOOD_DISTANCE_SHOOTER_TING_SWITCH_CASE = 120;
    public static double ALL_BALL_CONFIDENCE_THRESHOLD = 2;


    public int getQueuedShots() {
        return queuedShots;
    }

    public double getTurretAngle() {
        return turret.getCurrentAngle();
    }

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
        turret.applyOffset();
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

        switch (targetState) {
            case IDLE:
                feeder.set(Feeder.FeederStates.BLOCKING, true);

                if (queuedShots >= 1) {
                    if (flywheel.get() == Flywheel.FlyWheelStates.IDLE) flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
                    targetState = ShooterStates.PREPPING;
                    if (turret.get() != Turret.TurretStates.ODOM_TRACKING) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case PREPPING:
                calculateShotVectorAndUpdateTurret();
                double distance = turret.getDistance();

                if ((queuedShots >= 1 &&
                        flywheel.get() == Flywheel.FlyWheelStates.RUNNING &&
                        turret.isPIDInTolerance() &&
                        (robot.isAuto || distance > Common.MIN_SHOOTING_DISTANCE) &&
                        (distance <= 120 || turret.isReadyToShoot())) || inEmergency) {
                    inEmergency = false;
                    feeder.set(Feeder.FeederStates.RUNNING, true);
                    targetState = ShooterStates.RUNNING;
                    if (turret.get() != Turret.TurretStates.ODOM_TRACKING) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case RUNNING:
                calculateShotVectorAndUpdateTurret();
                flywheel.set(Flywheel.FlyWheelStates.RUNNING, true);
                feeder.set(Feeder.FeederStates.RUNNING, true);

                if (didShotOccur) {
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


    private void calculateShotVectorAndUpdateTurret() {
        //constants
        Vector robotToGoalVector = new Vector(turret.getDistance(), atan2(turret.getGoal().getY() - robot.drivetrain.getPose().getY(), turret.getGoal().getX() - robot.drivetrain.getPose().getX()));

        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), Hood.MAX,
                Hood.MIN);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        //get robot velocity and convert it into parallel and perpendicular components
        Vector robotVelocity = robot.drivetrain.poseTracker.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        //recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), Hood.MAX,
                Hood.MIN);

        flywheel.flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        if (!isHoodManual) hood.set(hoodAngle);

        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);

        turret.setTracking(Math.toDegrees(turretVelCompOffset));
//        double turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset);
//
//        if (turretAngle > 180) {
//            turretAngle -= 360;
//        }
    }

        public void printTelemetry() {
            turret.printTelemetry();
            flywheel.printTelemetry();
            feeder.printTelemetry();
            hood.printTelemetry();

        telemetry.addLine("SHOOTER");
        telemetry.addData("shooter state (ENUM):", targetState);
        telemetry.addData("queued shots (DOUBLE): ", queuedShots);
        telemetry.addData("did current drop? (BOOLEAN): ", didShotOccur);
    }
}
