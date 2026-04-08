
package org.firstinspires.ftc.teamcode.decode.subsystem;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.cbrt;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.teamcode.decode.control.Ranges;

/**
 * Solution to the circular-parabolic fused-arc kinematic shooting model for the FTC DECODE season <br>
 * Displacements are in inches (in) <br>
 * Angles are in radians (rad) <br>
 * Durations are in seconds (s) <br>
 *
 * @author Arshad Anas
 * @since 2025/09/18
 */
@Config
public final class KinematicsSolver {

    public static final Vector2 o_goal = new Vector2(5, -2.5);
    public static double
            r_rimClearance = 0.75,
            admissibleVerticalErrorAtGoal = 1,
            y_goal = 40,

            θ_launchMax = toRadians(48.5),
            θ_launchMin = θ_launchMax - toRadians((Hood.MAX - Hood.MIN) * (15.0/131) * (40.0/20)),
            v_launchMin = 0,
            v_launchMax = Flywheel.RPMToInchesPerSecond(Flywheel.MAX_RPM);

    private static final double
            a_G = -386.0886,
            o_turretForward = Common.TURRET_OFFSET_Y,
            y_rim = 38.75,
            r_ball = 2.5,
            r_wheel_physical = 1.5,
            r_compression = 6/25.4,
            r_wheel = r_wheel_physical - r_compression,
            c = r_wheel + r_ball,
            half_F = 141.5/2,
            rim_dist_comparison_error = 0.1;

    private static final Vector2
            s_wheel = new Vector2(3.64584291339,10.611220472440944),
            Center = new Vector2(half_F, half_F);

    private final Vector2
            // constant after init
            G = new Vector2(),

    // constant during solving
    s_turret = new Vector2(),
            v_turret = new Vector2(),
            unitTurretToGoal = new Vector2(),

    // modified during solving
    s0 = new Vector2(),
            s_launch = new Vector2(),
            unitLaunchPtToGoal = new Vector2(),
            v_relToGoal = new Vector2(),
            k = new Vector2(),
            s_goal = new Vector2(),
            s_rim = new Vector2(),
            v0 = new Vector2(),
            s_atGoal = new Vector2(),
            s_rimNearest = new Vector2(),
            s_rimTarget = new Vector2();

    private double m2, b, turretAngle;

    double θ_launch = (θ_launchMin + θ_launchMax) / 2,
            v_launch = (v_launchMin + v_launchMax) / 2,
            α_launch = 0;

    public void setAlliance(boolean isRedAlliance) {
        double i = isRedAlliance ? 1 : -1;

        G.set(Center).add((half_F - o_goal.x) * i, half_F + o_goal.y);

        Vector2
                A_rim = Center.sum(47.42197 * i, 70.04449),
                B_rim = A_rim.sum(15.85266 * i, -21.85619);

        m2 = (B_rim.y - A_rim.y) / (B_rim.x - A_rim.x);
        b = -m2 * A_rim.x + A_rim.y;
    }

    public void setRobotState(Pose pose, Vector velocity, double angVel) {
        double
                heading = pose.getHeading(),
                turretX = o_turretForward * cos(heading),
                turretY = o_turretForward * sin(heading);
        s_turret.set(pose.getX() + turretX, pose.getY() + turretY);
        v_turret.set(velocity.getXComponent() + angVel * -turretY, velocity.getYComponent() + angVel * turretX);
        unitTurretToGoal.set(G);
        unitTurretToGoal.subtract(s_turret);
        unitTurretToGoal.normalize();
        turretAngle = -unitTurretToGoal.getAngleBetween(heading);
    }

    public double getTurretAngle() {
        return turretAngle + α_launch;
    }

    private void setRobotState(double x, double y, double heading, double vx, double vy, double angVel) {
        Vector vRobot = new Vector();
        vRobot.setOrthogonalComponents(vx, vy);
        setRobotState(new Pose(x, y, heading), vRobot, angVel);
    }

    private void computeForwardKinematics() {
        double cos_θ_launch = cos(θ_launch), sin_θ_launch = sin(θ_launch);

        s0.set(s_wheel).add(-c*sin_θ_launch, c*cos_θ_launch);

        s_launch.set(unitTurretToGoal).rotate(α_launch).multiply(s0.x).add(s_turret);
        unitLaunchPtToGoal.set(G).subtract(s_launch).normalize();
        v_relToGoal.set(
                -v_turret.dot(unitLaunchPtToGoal),
                -v_turret.dot(-unitLaunchPtToGoal.y, unitLaunchPtToGoal.x*1)
        );

        double m1 = (G.y - s_launch.y) / (G.x - s_launch.x);
        k.x = (m1 * s_launch.x - s_launch.y + b) / (m1 - m2);
        k.y = m2 * k.x + b;

        s_goal.set(s_launch.distance(G) + s0.x, y_goal);
        s_rim.set(s_launch.distance(k) + s0.x, y_rim);

        v0.set(
                sqrt(v_launch*v_launch * cos_θ_launch*cos_θ_launch - v_relToGoal.y*v_relToGoal.y) - v_relToGoal.x,
                v_launch * sin_θ_launch
        );

        double t = tx(s_goal.x);
        s_atGoal.set(s_goal.x, s0.y + (v0.y + a_G/2 * t) * t);
    }

    private double tx(double x) {
        return (x - s0.x) / v0.x;
    }

    private static double W(double x, double A, double B, double C, double D) {
        return (((A/4*x + B/3)*x + C/2)*x + D)*x;
    }

    private void computeRimApproach(double vx, double vy) {

        double
                M = vy/vx,
                N = 2*a_G / (vx*vx),
                O = 2*M - N*s0.x,
                P = s_rim.y - s0.y + 0.25*s0.x*(2*M + O),

                A = 0.25*N*N,
                B = 0.75*N*O,
                C = 2 + 0.5*O*O - N*P,
                D = -2*s_rim.x - O*P,

                b3a = B / (3*A),
                Q = C/(3*A) - b3a*b3a,
                R = (b3a*C - D) / (2*A) - b3a*b3a*b3a,
                Q3 = Q*Q*Q,
                Dc = Q3 + R*R,

                x_nearest;

        if (Dc < 0) {
            double
                    theta = acos( R / sqrt(-Q3) ) / 3,
                    sQ = 2 * sqrt(-Q),
                    twoThirdsPi = 2 * PI / 3,
                    x1 = sQ * cos(theta) - b3a,
                    x2 = sQ * cos(theta + twoThirdsPi) - b3a,
                    x3 = sQ * cos(theta + 2*twoThirdsPi) - b3a,
                    d1 = W(x1, A, B, C, D),
                    d2 = W(x2, A, B, C, D),
                    d3 = W(x3, A, B, C, D);

            x_nearest = (d1 < d2) ?
                    (d1 < d3) ? x1 : x3 :
                    (d2 < d3) ? x2 : x3;

        } else if (Dc > 0) {
            double
                    sqrtD = sqrt(Dc),
                    S = cbrt(R + sqrtD),
                    T = cbrt(R - sqrtD);

            x_nearest = (S + T) - b3a;
        } else {
            double
                    cbrtR = cbrt(R),
                    x1 = 2*cbrtR - b3a,
                    x2 =   cbrtR - b3a,
                    d1 = W(x1, A, B, C, D),
                    d2 = W(x2, A, B, C, D);

            x_nearest = (d1 < d2) ? x1 : x2;
        }

        double dx = x_nearest - s0.x;
        s_rimNearest.set(x_nearest, 0.25*N*dx*dx + M*dx + s0.y);

        s_rimTarget.set(s_rimNearest).subtract(s_rim).setMagnitude(r_ball + r_rimClearance);
        if (s_rimTarget.y < 0)
            s_rimTarget.negate();
        s_rimTarget.add(s_rim);

    }

    private double v1(double cos_θ, double tan_θ) {
        double dx = s_goal.x - s0.x;
        return dx / ( cos_θ * sqrt( (2 / a_G) * (s_goal.y - s0.y - dx*tan_θ) ));
    }

    private double θ1(double v, boolean upper) {
        return θ1(v, upper, 0);
    }
    private double θ1(double v, boolean upper, double y_offset) {
        double i = v / (a_G * (s_goal.x - s0.x));
        return -atan( i*v + (upper ? -1 : 1) * sqrt(i*i*(v*v + 2*a_G*(s_goal.y - s0.y + y_offset)) - 1) );
    }

    private double θ_3pt(Vector2 target, double y_offset) {
        double
                dx = s_goal.x - s0.x,
                dy = s_goal.y - s0.y,
                tx = target.x - s0.x,
                ty = target.y - s0.y + y_offset;

        return atan(
                ( dx*dx*ty - tx*tx*dy ) /
                        ( dx*tx*(dx - tx)  )
        );
    }

    /**
     * After calling this method, convert {@link #v_launch} to RPM and use it as your wheel PID setpoint
     *
     * @return Whether the calculated launch will clear the goal rim.
     *         True = passes safely above the rim and reaches the goal point,
     *         within {@link #admissibleVerticalErrorAtGoal} inches vertical error
     */
    public boolean calculateTarget_v_θ_α() {
        θ_launch = (θ_launchMin + θ_launchMax) / 2;
        α_launch = 0;
        double θi, cos_θi = 0, sin_θi = 0, tan_θi, vi = 0, vf, θf, α;

        for (int j = 0; j < 5; j++) {
            computeForwardKinematics();

            if (j == 0) {
                θi = θ_3pt(s_rim, r_ball + r_rimClearance);
                sin_θi = sin(θi);
                cos_θi = cos(θi);
                tan_θi = sin_θi / cos_θi;
                vi = v1(cos_θi, tan_θi);
            }

            computeRimApproach(vi*cos_θi, vi*sin_θi);
            θi = θ_3pt(s_rimTarget, 0);

            int n = 3;
            for (int i = 0; i < n; i++) {
                sin_θi = sin(θi);
                cos_θi = cos(θi);
                tan_θi = sin_θi / cos_θi;
                vi = v1(cos_θi, tan_θi);

                vf = sqrt(vi * vi + v_relToGoal.getMagnitudeSquared() + 2 * v_relToGoal.x * vi * cos_θi);
                θf = asin(vi * sin_θi / vf);
                α = atan(v_relToGoal.y / (vi * cos_θi + v_relToGoal.x));

                if (i + 1 >= n || θ_launchMin <= θf && θf <= θ_launchMax) {
                    θ_launch = θf;
                    v_launch = vf;
                    α_launch = α;
                    break;
                }

                double θ_clipped = Ranges.clip(θf, θ_launchMin, θ_launchMax);
                double cos_clipped = cos(θ_clipped), sin_clipped = sin(θ_clipped);
                θi = atan(
                        vf * sin_clipped /
                                ( sqrt(vf * vf * cos_clipped * cos_clipped - v_relToGoal.y * v_relToGoal.y) - v_relToGoal.x )
                );
            }
        }

        v_launch = Ranges.clip(v_launch, v_launchMin, v_launchMax);

        computeForwardKinematics();
        computeRimApproach(v0.x, v0.y);
        return
                v_launch >= v_launchMin && v_launch < v_launchMax &&
                        s_rimNearest.y >= s_rim.y &&
                        s_rimNearest.distance(s_rim) >= r_ball + r_rimClearance - rim_dist_comparison_error &&
                        abs(s_atGoal.y - s_goal.y) <= admissibleVerticalErrorAtGoal;
    }

    /**
     * @return Distance to rim at nearest approach. NaN if launch collides with goal wall or cannot reach goal point
     */
    public double calculateTarget_θ_α(double currentV, boolean upper) {
        v_launch = currentV;
        θ_launch = (θ_launchMin + θ_launchMax) / 2;
        α_launch = 0;
        computeForwardKinematics();         // determine flight distance for drag correction

        double θi, cos_θi, sin_θi, vi, vf, θf, α;

        for (int j = 0; j < 5; j++) {
            computeForwardKinematics();

            vi = v0.getMagnitude();
            θi = θ1(vi, upper);

            if (Double.isNaN(θi)) {
                θi = θ1(vi, upper, -admissibleVerticalErrorAtGoal);
                if (Double.isNaN(θi))
                    return θi;
            }

            int n = 2;
            for (int i = 0; i < n; i++) {
                sin_θi = sin(θi);
                cos_θi = cos(θi);

                vf = v_launch;
                θf = asin(vi * sin_θi / vf);
                α = atan(v_relToGoal.y / (vi * cos_θi + v_relToGoal.x));

                if (i + 1 >= n || θ_launchMin <= θf && θf <= θ_launchMax) {
                    θ_launch = θf;
                    α_launch = α;
                    break;
                }

                double θ_clipped = Ranges.clip(θf, θ_launchMin, θ_launchMax);
                double cos_clipped = cos(θ_clipped), sin_clipped = sin(θ_clipped);
                θi = atan(
                        vf * sin_clipped /
                                ( sqrt(vf * vf * cos_clipped * cos_clipped - v_relToGoal.y * v_relToGoal.y) - v_relToGoal.x )
                );
            }
        }

        computeForwardKinematics();
        computeRimApproach(v0.x, v0.y);
        double rimDist = s_rimNearest.distance(s_rim);
        return
                s_rimNearest.y >= s_rim.y &&
                        rimDist >= r_ball + r_rimClearance - rim_dist_comparison_error &&
                        abs(s_atGoal.y - s_goal.y) <= admissibleVerticalErrorAtGoal ?
                        rimDist :
                        Double.NaN;
    }

    /**
     * After calling this method, convert {@link #θ_launch} from launch angle, in radians, to hood
     * servo angle in degrees. {@link #α_launch} is the offset, in radians, of the turret CCW away
     * from the vector in the direction of the goal.
     *
     * @return Whether the calculated launch will clear the goal rim.
     *         True = passes safely above the rim and reaches the goal point,
     *         within {@link #admissibleVerticalErrorAtGoal} inches vertical error
     */
    public boolean calculateTarget_θ_α(double currentV) {
        double
                d1 = calculateTarget_θ_α(currentV, true),
                v1 = v_launch,
                θ1 = θ_launch,
                α1 = α_launch,
                d2 = calculateTarget_θ_α(currentV, false),
                v2 = v_launch,
                θ2 = θ_launch,
                α2 = α_launch;

        boolean valid1 = !Double.isNaN(d1), valid2 = !Double.isNaN(d2);

        if (!valid1 && !valid2) return false;

        if (valid1 && (!valid2 || d1 < d2)) {
            v_launch = v1;
            θ_launch = θ1;
            α_launch = α1;
        } else {
            v_launch = v2;
            θ_launch = θ2;
            α_launch = α2;
        }
        return true;
    }

    void printTelemetry() {
        Common.dashTelemetry.addData("turret pos (in): ", s_turret);
        Common.dashTelemetry.addData("goal pos (in): ", G);
        Common.dashTelemetry.addData("turret to goal distance (in): ", s_turret.distance(G));
        Common.dashTelemetry.addData("unit vector to goal: ", unitTurretToGoal);
    }

    public String resultsToString() {
        return "θ_launch (Deg): " + Math.toDegrees(θ_launch) + "\nv_launch: " + v_launch + "\nα_launch: " + α_launch;
    }
    public void printResults() {
        System.out.println(resultsToString()+"\n");
    }

    /**
     * Use <a href="https://www.desmos.com/calculator/agavajtz5x">this Desmos</a> to visualize your test cases:
     */
    public static void main(String[] args) {
        new Vector2(3,3).normalize();
        new Pose(0, 0, 0);

        KinematicsSolver solver = new KinematicsSolver();
        solver.setAlliance(true);

        solver.setRobotState(100,100, -3.12, 0.5, 95, 0.13);
        System.out.println(solver.calculateTarget_v_θ_α());
        solver.printResults();

        solver.setRobotState(70,17, -0.05, -50.4, 0.5, 0.13);
        System.out.println(solver.calculateTarget_θ_α(200));
        solver.printResults();
    }

}
