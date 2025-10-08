package org.firstinspires.ftc.teamcode.decode.util;

import com.pedropathing.geometry.Pose;

public class ShooterZoneChecker {
    private final double
        ROBOT_WIDTH = 14.043,
        ROBOT_LENGTH = 16.3;

    private Pose[] rectangle = generateRectangle(0,0,ROBOT_WIDTH,ROBOT_LENGTH,0);

    public static Pose[] farTriangle = {
            new Pose(50, 0),
            new Pose(72, 23),
            new Pose(95, 0)
    };

    public static Pose[] closeTriangle = {
            new Pose(0, 142),
            new Pose(72, 73),
            new Pose(142, 142)
    };

    public void setRectangle(double x, double y, double heading) {
        rectangle = generateRectangle(x, y, ROBOT_WIDTH, ROBOT_LENGTH, heading);
    }

    // Check if Pose P is inside triangle ABC using barycentric coordinates
    public static boolean PoseInTriangle(Pose p, Pose a, Pose b, Pose c) {
        double denominator = ((b.getY() - c.getY())*(a.getX() - c.getX()) + (c.getX() - b.getX())*(a.getY() - c.getY()));
        if (denominator == 0) return false; // Degenerate triangle
        double alpha = ((b.getY() - c.getY())*(p.getX() - c.getX()) + (c.getX() - b.getX())*(p.getY() - c.getY())) / denominator;
        double beta = ((c.getY() - a.getY())*(p.getX() - c.getX()) + (a.getX() - c.getX())*(p.getY() - c.getY())) / denominator;
        double gamma = 1.0 - alpha - beta;

        return alpha >= 0 && beta >= 0 && gamma >= 0;
    }

    // Check if Pose P is inside a convex quad by splitting into two triangles
    public static boolean PoseInQuad(Pose p, Pose[] quad) {
        return PoseInTriangle(p, quad[0], quad[1], quad[2]) ||
                PoseInTriangle(p, quad[0], quad[2], quad[3]);
    }

    // Check if line segments AB and CD intersect
    public static boolean segmentsIntersect(Pose a, Pose b, Pose c, Pose d) {
        return ccw(a, c, d) != ccw(b, c, d) && ccw(a, b, c) != ccw(a, b, d);
    }

    private static boolean ccw(Pose a, Pose b, Pose c) {
        return (c.getY() - a.getY())*(b.getX() - a.getX()) > (b.getY() - a.getY())*(c.getX() - a.getX());
    }

    // Main check for rectangle-triangle intersection
    public boolean checkRectangleTriangleIntersection(Pose[] tri) {
        // 1. Any rectangle Pose inside triangle?
        for (Pose p : rectangle) {
            if (PoseInTriangle(p, tri[0], tri[1], tri[2])) return true;
        }

        // 2. Any triangle Pose inside rectangleangle?
        for (Pose p : tri) {
            if (PoseInQuad(p, rectangle)) return true;
        }

        // 3. Any edge intersection between rectangleangle and triangle?
        for (int i = 0; i < 4; i++) {
            Pose r1 = rectangle[i];
            Pose r2 = rectangle[(i + 1) % 4];
            for (int j = 0; j < 3; j++) {
                Pose t1 = tri[j];
                Pose t2 = tri[(j + 1) % 3];
                if (segmentsIntersect(r1, r2, t1, t2)) return true;
            }
        }

        return false;
    }

    /**
     * Generate 4 Poses of a rotated rectangle.
     * @param cx     Center X
     * @param cy     Center Y
     * @param width  Width of rectangle (short side)
     * @param length Length of rectangle (long side)
     * @param headingDegrees Rotation angle in radians (0 is along +X axis)
     * @return Array of 4 Poses in clockwise order
     */
    public static Pose[] generateRectangle(double cx, double cy, double width, double length, double headingDegrees) {
        double dx = length / 2.0;
        double dy = width / 2.0;
        double headingRadians = Math.toDegrees(headingDegrees);
        // Unit vectors along rotated axes
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);

        // Corner offsets from center
        Pose offset1 = new Pose(-dx * cos + dy * sin, -dx * sin - dy * cos);
        Pose offset2 = new Pose( dx * cos + dy * sin,  dx * sin - dy * cos);
        Pose offset3 = new Pose( dx * cos - dy * sin,  dx * sin + dy * cos);
        Pose offset4 = new Pose(-dx * cos - dy * sin, -dx * sin + dy * cos);

        return new Pose[] {
                new Pose(cx + offset1.getX(), cy + offset1.getY()),
                new Pose(cx + offset2.getX(), cy + offset2.getY()),
                new Pose(cx + offset3.getX(), cy + offset3.getY()),
                new Pose(cx + offset4.getX(), cy + offset4.getY())
        };
    }
}
