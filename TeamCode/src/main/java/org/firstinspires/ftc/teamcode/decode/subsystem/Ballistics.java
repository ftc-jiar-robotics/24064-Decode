package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Projectile-motion solver + velocity compensation for shooting while moving.
 *
 * Units:
 *  - Field positions: inches
 *  - Velocities: inches/sec (Pedro follower / pose tracker velocity components)
 *  - Angles: radians (solver). Convert to degrees only when commanding turret/servo domains.
 */
@Configurable
public final class Ballistics {

    private Ballistics() {}

    /** Gravity in inches/sec^2 (32.174 ft/s^2 * 12 in/ft). */
    public static double G_IN_PER_S2 = 32.174 * 12.0;

    /** Subtract from geometric distance so we solve for a point slightly "inside" the goal. */
    public static double PASS_THROUGH_POINT_RADIUS = ShooterConstants.PASS_THROUGH_POINT_RADIUS;

    /** Height of the goal above the launch point (in). */
    public static double SCORE_HEIGHT = ShooterConstants.SCORE_HEIGHT;

    /**
     * Required trajectory angle at the goal (radians).
     * Negative means the ball is descending into the goal.
     */
    public static double SCORE_ANGLE_RAD = ShooterConstants.SCORE_ANGLE;

    /** Clamp physical launch angle (deg) after compensation to keep it achievable. */
    public static boolean CLAMP_LAUNCH_ANGLE = true;

    /** These are *physical* launch angles above horizontal, not servo angles. Tune to your shooter. */
    public static double LAUNCH_ANGLE_MIN_DEG = 15.0;
    public static double LAUNCH_ANGLE_MAX_DEG = 75.0;

    public static class Solution {
        public final boolean valid;

        // Geometry
        public final double distanceToGoal;     // in
        public final double xEffective;         // in (distance - PASS_THROUGH_POINT_RADIUS)

        // Robot velocity components in goal coordinates (field frame)
        // NOTE: "parallelComp" matches your screenshot: it is NEGATIVE of robot's component toward goal.
        public final double parallelComp;       // in/s  (= -v_toward_goal)
        public final double perpendicularComp;  // in/s  (left-positive around goal)

        // Uncompensated solution (stationary assumption)
        public final double launchAngleRad;     // rad (alpha)
        public final double launchSpeed;        // in/s (v0)
        public final double flightTime;         // s

        // Velocity-compensated solution
        public final double launchAngleRadComp; // rad
        public final double launchSpeedComp;    // in/s
        public final double xEffectiveComp;     // in (new horizontal "effective" distance)

        // Turret lead offset (rad). Add/subtract in your turret aiming logic.
        public final double turretLeadOffsetRad;

        private Solution(
                boolean valid,
                double distanceToGoal,
                double xEffective,
                double parallelComp,
                double perpendicularComp,
                double launchAngleRad,
                double launchSpeed,
                double flightTime,
                double launchAngleRadComp,
                double launchSpeedComp,
                double xEffectiveComp,
                double turretLeadOffsetRad
        ) {
            this.valid = valid;
            this.distanceToGoal = distanceToGoal;
            this.xEffective = xEffective;
            this.parallelComp = parallelComp;
            this.perpendicularComp = perpendicularComp;
            this.launchAngleRad = launchAngleRad;
            this.launchSpeed = launchSpeed;
            this.flightTime = flightTime;
            this.launchAngleRadComp = launchAngleRadComp;
            this.launchSpeedComp = launchSpeedComp;
            this.xEffectiveComp = xEffectiveComp;
            this.turretLeadOffsetRad = turretLeadOffsetRad;
        }
    }

    /**
     * Solve for projectile launch angle/speed and apply motion compensation.
     *
     * @param turretPos Field pose of the turret exit point (in)
     * @param goal      Field pose of the goal (in)
     * @param vx        Robot velocity X component (in/s) in field frame
     * @param vy        Robot velocity Y component (in/s) in field frame
     */
    public static Solution solve(Pose turretPos, Pose goal, double vx, double vy) {
        final double dx = goal.getX() - turretPos.getX();
        final double dy = goal.getY() - turretPos.getY();

        final double dist = Math.hypot(dx, dy);
        if (dist < 1e-6) {
            return new Solution(false, dist, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }

        // Effective "x" distance used in the projectile equations
        final double x = dist - PASS_THROUGH_POINT_RADIUS;
        if (x <= 1.0) {
            return new Solution(false, dist, x, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        }

        // Goal-direction unit vector (field frame)
        final double ux = dx / dist;
        final double uy = dy / dist;

        // "Left" perpendicular unit vector (field frame)
        final double px = -uy;
        final double py = ux;

        // Robot velocity components in the goal coordinate frame
        final double vTowardGoal = (vx * ux + vy * uy);         // + = moving toward goal
        final double parallelComp = -vTowardGoal;               // matches your code: -cos(theta)*|v|
        final double perpendicularComp = (vx * px + vy * py);   // matches sin(theta)*|v|

        // 1) Uncompensated launch angle using required entry angle at the goal.
        final double a = SCORE_ANGLE_RAD;
        final double alpha = Math.atan(2.0 * SCORE_HEIGHT / x - Math.tan(a));

        // 2) Uncompensated launch speed from projectile motion (no air resistance).
        final double cosA = Math.cos(alpha);
        final double denom = 2.0 * cosA * cosA * (x * Math.tan(alpha) - SCORE_HEIGHT);

        if (Math.abs(denom) < 1e-9 || denom <= 0) {
            return new Solution(false, dist, x, parallelComp, perpendicularComp, alpha, 0, 0, 0, 0, 0, 0);
        }

        final double v0 = Math.sqrt(G_IN_PER_S2 * x * x / denom);

        // Flight time to cover x horizontally (relative-to-robot horizontal component)
        final double vxBall = v0 * cosA;
        final double t = x / vxBall;

        // 3) Velocity compensation (matches the logic in your screenshot).
        // ivr = x/time + parallelComponent  -> here: vxComp = vxBall + parallelComp
        final double vxComp = vxBall + parallelComp;

        // nvr = sqrt(ivr^2 + perpendicular^2)
        final double vxNewMag = Math.hypot(vxComp, perpendicularComp);

        // ndr = nvr * time -> xComp
        final double xComp = vxNewMag * t;

        // Keep vertical component from the original solution (vz = v0*sin(alpha))
        final double vz = v0 * Math.sin(alpha);

        // New physical launch angle from (vz, nvr)
        double alphaComp = Math.atan2(vz, vxNewMag);

        if (CLAMP_LAUNCH_ANGLE) {
            double deg = Math.toDegrees(alphaComp);
            deg = clamp(deg, LAUNCH_ANGLE_MIN_DEG, LAUNCH_ANGLE_MAX_DEG);
            alphaComp = Math.toRadians(deg);
        }

        // Recompute launch speed needed to still hit the goal with (xComp, alphaComp)
        final double cosC = Math.cos(alphaComp);
        final double denom2 = 2.0 * cosC * cosC * (xComp * Math.tan(alphaComp) - SCORE_HEIGHT);

        if (Math.abs(denom2) < 1e-9 || denom2 <= 0) {
            return new Solution(false, dist, x, parallelComp, perpendicularComp, alpha, v0, t, alphaComp, 0, xComp, 0);
        }

        final double vComp = Math.sqrt(G_IN_PER_S2 * xComp * xComp / denom2);

        // 4) Turret lead offset (atan(perp / ivr)) but safe with atan2
        final double turretOffset = Math.atan2(perpendicularComp, vxComp);

        return new Solution(true, dist, x, parallelComp, perpendicularComp, alpha, v0, t, alphaComp, vComp, xComp, turretOffset);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
