package org.firstinspires.ftc.teamcode.decode.subsystem;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;

import org.dyn4j.collision.narrowphase.Gjk;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Transform;

public enum LaunchZone {
    NONE,
    NEAR,
    FAR;

    public static final double
            LENGTH_DT = 13.85827,
            LENGTH_INTAKE = 2.91774,
            WIDTH_TOTAL_IN = 14.27953,
            LENGTH_TOTAL_IN = LENGTH_DT + LENGTH_INTAKE,
            FORWARD_OFFSET_IN = LENGTH_INTAKE/2,
            SIZE_FIELD = 141.5,
            SIZE_TILE = SIZE_FIELD/6;

    private static final double NEAR_CX = SIZE_FIELD / 2.0, NEAR_CY = SIZE_FIELD;
    private static final double FAR_CX  = SIZE_FIELD / 2.0, FAR_CY  = 0.0;
    private static final double ZONE_ANGLE = toRadians(45);
    private static final double nearHalfSize, farHalfSize;

    private static final Transform nearZonePosition, farZonePosition, robotPose = new Transform();
    private static final Rectangle nearZoneRect, farZoneRect, robotRect;
    private static final Gjk collisionSolver = new Gjk();

    static {
        double nearZoneSize = SIZE_TILE * Math.sqrt(2) * 3;
        double farZoneSize = SIZE_TILE * Math.sqrt(2);
        nearHalfSize = nearZoneSize / 2.0;
        farHalfSize  = farZoneSize  / 2.0;

        nearZoneRect = new Rectangle(nearZoneSize, nearZoneSize);
        farZoneRect = new Rectangle(farZoneSize, farZoneSize);

        robotRect = new Rectangle(LENGTH_TOTAL_IN, WIDTH_TOTAL_IN * 1);

        nearZonePosition = new Transform();
        farZonePosition = new Transform();

        nearZonePosition.rotate(toRadians(45));
        nearZonePosition.translate(SIZE_FIELD / 2, SIZE_FIELD);

        farZonePosition.rotate(toRadians(45));
        farZonePosition.translate(SIZE_FIELD / 2, 0);
    }

    /**
     * Returns the point on the boundary of the nearest launch zone the robot will reach,
     * given its current position and direction of travel from the velocity vector.
     * If the travel direction does not intersect either zone, returns the closest boundary
     * point on whichever zone is nearer to the robot.
     */
    public static Pose getInterceptOrClosestPoint() {
        Pose robotPos = Common.robot.drivetrain.getPose();

        // Already inside a zone — return current pose directly
        if (getCurrentZone(robotPos) != NONE) return robotPos;

        double rx = robotPos.getX(), ry = robotPos.getY();

        // Find closest boundary point on each zone, then aim toward whichever is nearer
        Pose nearClosest = closestBoundaryPoint(rx, ry, NEAR_CX, NEAR_CY, nearHalfSize);
        Pose farClosest  = closestBoundaryPoint(rx, ry, FAR_CX,  FAR_CY,  farHalfSize);
        Pose target = dist2(rx, ry, nearClosest) <= dist2(rx, ry, farClosest) ? nearClosest : farClosest;

        // Cast ray from robot toward that closest point to find the entry intersection
        double dx = target.getX() - rx, dy = target.getY() - ry;
        boolean isNear = target == nearClosest;
        Pose hit = isNear
                ? rayRectIntercept(rx, ry, dx, dy, NEAR_CX, NEAR_CY, nearHalfSize)
                : rayRectIntercept(rx, ry, dx, dy, FAR_CX,  FAR_CY,  farHalfSize);

        return hit != null ? hit : target;
    }

    /**
     * Ray-rectangle intersection in world space.
     * The rectangle is a square with half-side {@code halfSize} centered at (cx, cy), rotated 45°.
     * Returns the first hit point in the travel direction, or null if there is none.
     */
    private static Pose rayRectIntercept(
            double rx, double ry, double dx, double dy,
            double cx, double cy, double halfSize) {

        // Transform ray into the rectangle's local (unrotated) frame
        double relX = rx - cx, relY = ry - cy;
        double cosN = cos(-ZONE_ANGLE), sinN = sin(-ZONE_ANGLE);
        double lox = relX * cosN - relY * sinN;
        double loy = relX * sinN + relY * cosN;
        double ldx = dx * cosN - dy * sinN;
        double ldy = dx * sinN + dy * cosN;

        // Slab intersection with AABB [-h, h] x [-h, h]
        double h = halfSize;
        double txMin, txMax, tyMin, tyMax;
        if (Math.abs(ldx) < 1e-9) {
            if (Math.abs(lox) > h) return null;
            txMin = Double.NEGATIVE_INFINITY; txMax = Double.POSITIVE_INFINITY;
        } else {
            txMin = (-h - lox) / ldx; txMax = (h - lox) / ldx;
            if (txMin > txMax) { double tmp = txMin; txMin = txMax; txMax = tmp; }
        }
        if (Math.abs(ldy) < 1e-9) {
            if (Math.abs(loy) > h) return null;
            tyMin = Double.NEGATIVE_INFINITY; tyMax = Double.POSITIVE_INFINITY;
        } else {
            tyMin = (-h - loy) / ldy; tyMax = (h - loy) / ldy;
            if (tyMin > tyMax) { double tmp = tyMin; tyMin = tyMax; tyMax = tmp; }
        }

        double tMin = Math.max(txMin, tyMin);
        double tMax = Math.min(txMax, tyMax);
        if (tMin > tMax || tMin < 0) return null; // no hit, or entry is behind

        double lpx = lox + tMin * ldx, lpy = loy + tMin * ldy;

        // Transform hit point back to world space
        double cosF = cos(ZONE_ANGLE), sinF = sin(ZONE_ANGLE);
        return new Pose(lpx * cosF - lpy * sinF + cx,
                lpx * sinF + lpy * cosF + cy, 0);
    }

    /**
     * Closest point on the boundary of a 45°-rotated square (center cx,cy, half-side halfSize)
     * to the query point (px, py).
     */
    private static Pose closestBoundaryPoint(
            double px, double py, double cx, double cy, double halfSize) {

        double relX = px - cx, relY = py - cy;
        double cosN = cos(-ZONE_ANGLE), sinN = sin(-ZONE_ANGLE);
        double lx = relX * cosN - relY * sinN;
        double ly = relX * sinN + relY * cosN;

        double h = halfSize;
        double clampedX, clampedY;

        if (Math.abs(lx) > h || Math.abs(ly) > h) {
            // Outside — clamp to nearest face
            clampedX = Math.max(-h, Math.min(h, lx));
            clampedY = Math.max(-h, Math.min(h, ly));
        } else {
            // Inside — project to nearest edge
            double toLeft = lx + h, toRight = h - lx;
            double toBot  = ly + h, toTop   = h - ly;
            double minD = Math.min(Math.min(toLeft, toRight), Math.min(toBot, toTop));
            clampedX = lx; clampedY = ly;
            if      (minD == toLeft)  clampedX = -h;
            else if (minD == toRight) clampedX =  h;
            else if (minD == toBot)   clampedY = -h;
            else                      clampedY =  h;
        }

        double cosF = cos(ZONE_ANGLE), sinF = sin(ZONE_ANGLE);
        return new Pose(clampedX * cosF - clampedY * sinF + cx,
                clampedX * sinF + clampedY * cosF + cy, 0);
    }

    private static double dist2(double rx, double ry, Pose p) {
        double dx = p.getX() - rx, dy = p.getY() - ry;
        return dx * dx + dy * dy;
    }

    static LaunchZone getCurrentZone(Pose currentPose) {

        robotPose.identity();
        robotPose.translate(FORWARD_OFFSET_IN, 0);
        robotPose.rotate(currentPose.getHeading());
        robotPose.translate(currentPose.getX(), currentPose.getY());

        return
                collisionSolver.detect(robotRect, robotPose, nearZoneRect, nearZonePosition) ? NEAR :
                        collisionSolver.detect(robotRect, robotPose, farZoneRect, farZonePosition) ? FAR :
                                NONE;
    }

    public static void main(String... args) {

        Pose[] poses = {
                new Pose(103.05710814094775, 75.76184690157962, toRadians(73)), // NONE
                new Pose(45.04407033129976, 84.21034755347013, toRadians(132)), // NONE

                new Pose(44.480836954507055, 84.58583647133193, toRadians(169)), // NEAR
                new Pose(123.30743288838414, 111.52020860495438, toRadians(169)), // NEAR
                new Pose(71.67770668238674, 64.95958279009125, toRadians(180)), // NEAR
                new Pose(106.78592050246495, 93.30899608865711, toRadians(136)), // NEAR

                new Pose(72.42868451811033, 29.475880052151236, toRadians(180)), // FAR
                new Pose(52.52777187143498, 14.831812255541072, toRadians(152)), // FAR

                new Pose(89.888919198684, 18.023468057366358, toRadians(136)), // NONE
        };

        for (Pose pose : poses)
            System.out.println(LaunchZone.getCurrentZone(pose));
    }
}