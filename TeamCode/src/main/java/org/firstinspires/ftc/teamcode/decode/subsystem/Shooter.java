package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.ANG_VELOCITY_MULTIPLER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
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
                turret.clearExternalLeadDeg();

                double distanceI = turret.getDistance();
//                if (!isHoodManual) hood.set(Hood.MIN);
                if (!isHoodManual) {
                    if (distanceI <= HOOD_DISTANCE_SHOOTER_TING_SWITCH_CASE) {
                        hood.set(hood.getHoodAngleWithDistance(distanceI), true);
                    } else {
                        hood.set(hood.getHoodAngleWithRPM(flywheel.getCurrentRPMSmooth()), true);
                    }
                }

                if (queuedShots >= 1) {
                    if (flywheel.get() == Flywheel.FlyWheelStates.IDLE) flywheel.set(Flywheel.FlyWheelStates.ARMING, true);
                    targetState = ShooterStates.PREPPING;
                    if (turret.get() != Turret.TurretStates.ODOM_TRACKING) turret.set(Turret.TurretStates.ODOM_TRACKING, true);
                }
                break;
            case PREPPING:
                double distance = turret.getDistance();
                if (USE_PROJECTILE_COMP) {
                    ShotComp comp = calculateShotCompFromVideoMath();
                    distance = comp.compensatedDistance;
                    turret.setExternalLeadDeg(comp.turretLeadDeg);
                } else {
                    turret.clearExternalLeadDeg();
                }
                if (!isHoodManual) {
                    if (distance <= HOOD_DISTANCE_SHOOTER_TING_SWITCH_CASE) {
                        hood.set(hood.getHoodAngleWithDistance(distance), true);
                    } else {
                        hood.set(hood.getHoodAngleWithRPM(flywheel.getCurrentRPMSmooth()), true);
                    }
                }

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
//                if (turret.isNotStable() || flywheel.isNotStable()) targetState = ShooterStates.PREPPING;

                if (!isHoodManual) {
                    double distanceR = turret.getDistance();
                    if (USE_PROJECTILE_COMP) {
                        ShotComp comp = calculateShotCompFromVideoMath();
                        distanceR = comp.compensatedDistance;
                        turret.setExternalLeadDeg(comp.turretLeadDeg);
                    } else {
                        turret.clearExternalLeadDeg();
                    }
                    if (distanceR <= HOOD_DISTANCE_SHOOTER_TING_SWITCH_CASE) {
                        hood.set(hood.getHoodAngleWithDistance(distanceR), true);
                    } else {
                        hood.set(hood.getHoodAngleWithRPM(flywheel.getCurrentRPMSmooth()), true);
                    }
                }

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

    private double vx;
    private double vy;
    private double omega;
    private double ax;
    private double ay;
    private double alpha;

    private Pose currentPose, predictedPose;
    public Pose getPredictedPose(double timeToShoot) {
        currentPose = robot.drivetrain.getPose();

        if (targetState != ShooterStates.RUNNING) {
            predictedPose = currentPose;
            return currentPose;
        }
//        double distanceInches = turret.getDistance();
//        double airtime = Common.getAirtimeForDistance(distanceInches)
        vx = robot.drivetrain.getVelocity().getXComponent();
        vy = robot.drivetrain.getVelocity().getYComponent();
        omega = robot.drivetrain.getAngularVelocity() * ANG_VELOCITY_MULTIPLER;
        ax = robot.drivetrain.getAcceleration().getXComponent();
        ay = robot.drivetrain.getAcceleration().getYComponent();
        // ay (no accel)
        alpha = 0;

        double
                // Predict velocity at shot time (accounts for accel if available)
                futureVx = vx + ax * timeToShoot,
                futureVy = vy + ay * timeToShoot,
                futureOmega = omega + alpha * timeToShoot,

                // Average velocity over interval
                avgVx = (vx + futureVx) / 2.0,
                avgVy = (vy + futureVy) / 2.0,
                avgOmega = (omega + futureOmega) / 2.0,

                // Displacement = average velocity × time
                dx = avgVx * timeToShoot,
                dy = avgVy * timeToShoot,
                dh = avgOmega * timeToShoot;

        // Return new predicted pose (in inches and radians)
        predictedPose = new Pose(
                currentPose.getX() + dx,
                currentPose.getY() + dy,
                currentPose.getHeading() + dh);
        return robot.isRobotMoving() ? predictedPose : currentPose;
    }


    // ======================= PROJECTILE / MOVING SHOT COMP (VIDEO MATH) =======================
    public static boolean USE_PROJECTILE_COMP = true;

    // if lead points wrong direction, flip to -1
    public static double LEAD_SIGN = 1.0;

    // gravity (in/s^2)
    public static double G = 32.174 * 12.0;

    private static class ShotComp {
        final double compensatedDistance;   // "ndr + passThrough"
        final double turretLeadDeg;         // degrees to add to turret aim
        final double hoodAngleRad;          // physical launch angle (for debugging)
        final double launchSpeed;           // physical initial speed (for debugging)

        ShotComp(double compensatedDistance, double turretLeadDeg, double hoodAngleRad, double launchSpeed) {
            this.compensatedDistance = compensatedDistance;
            this.turretLeadDeg = turretLeadDeg;
            this.hoodAngleRad = hoodAngleRad;
            this.launchSpeed = launchSpeed;
        }
    }
    
    private ShotComp calculateShotCompFromVideoMath() {
        // predicted pose so comp matches your LAUNCH_DELAY pipeline
        Pose predicted = isFuturePoseOn ? getPredictedPose(Turret.LAUNCH_DELAY) : robot.drivetrain.getPose();

        Pose turretPosLocal = Turret.calculateTurretPosition(
                predicted,
                Math.toDegrees(predicted.getHeading()),
                -Common.TURRET_OFFSET_Y
        );

        // goal pose from turret (already alliance-aware)
        Pose goal = turret.getGoalPose(); // <-- you will add this 1-line getter in Turret (see below)

        // robot->goal vector
        double dx = goal.getX() - turretPosLocal.getX();
        double dy = goal.getY() - turretPosLocal.getY();
        double robotToGoalTheta = Math.atan2(dy, dx);
        double robotToGoalMag = Math.hypot(dx, dy);

        // ---------------- constants (video) ----------------
        double g = G;
        double x = robotToGoalMag - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

        if (x <= 1e-6) {
            return new ShotComp(robotToGoalMag, 0.0, 0.0, 0.0);
        }

        // ---------------- initial launch components (video) ----------------
        double hoodAngle = com.pedropathing.math.MathFunctions.clamp(
                Math.atan(2 * y / x - Math.tan(a)),
                Math.toRadians(75), // clamp physical alpha (tune if needed)
                Math.toRadians(15)
        );

        double denom = (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y));
        if (denom <= 1e-9) {
            return new ShotComp(robotToGoalMag, 0.0, hoodAngle, 0.0);
        }

        double flywheelSpeed = Math.sqrt(g * x * x / denom); // physical v0 (in/s)

        // ---------------- robot velocity -> parallel / perpendicular (video) ----------------
        double vx = robot.drivetrain.getVelocity().getXComponent();
        double vy = robot.drivetrain.getVelocity().getYComponent();

        double robotVelTheta = Math.atan2(vy, vx);
        double robotVelMag = Math.hypot(vx, vy);

        double coordinateTheta = robotVelTheta - robotToGoalTheta;

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelMag;
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelMag;

        // ---------------- velocity compensation variables (video) ----------------
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));

        double ivr = x / time + parallelComponent; // effective horizontal speed in goal direction
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time; // EFFECTIVE horizontal distance ("ndr" in video)

        // ---------------- recalculate launch components (video) ----------------
        hoodAngle = com.pedropathing.math.MathFunctions.clamp(
                Math.atan(vz / nvr),
                Math.toRadians(75),
                Math.toRadians(15)
        );

        double denom2 = (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y));
        if (denom2 > 1e-9) {
            flywheelSpeed = Math.sqrt(g * ndr * ndr / denom2);
        }

        // ---------------- update turret (video) ----------------
        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr); // atan2 for safety

        // This is the key output you actually use in YOUR code:
        double effectiveDistance = ndr + ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double leadDeg = LEAD_SIGN * Math.toDegrees(turretVelCompOffset);

        return new ShotComp(effectiveDistance, leadDeg, hoodAngle, flywheelSpeed);
    }
// ======================= END PROJECTILE / MOVING SHOT COMP =======================

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
    }
}
