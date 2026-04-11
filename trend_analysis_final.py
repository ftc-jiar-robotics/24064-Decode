#!/usr/bin/env python3
"""
Trend Analysis: KinematicsSolver vs Curve Fitting
Robot is stationary (vx=vy=angVel=0) on a 144x144 field

Based on:
- Hood.java: curve fitting equations
- Flywheel.java: RPM calculations
- KinematicsSolver.java: projectile physics
"""

import math

class Constants:
    """Physical and mechanical constants from the Java code"""
    # Field
    GOAL_X = 72
    GOAL_Y = 144

    # Hood servo limits
    HOOD_MIN = 68
    HOOD_MAX = 180
    HOOD_PHYSICAL_MAX = 200

    # Launch angle limits (from KinematicsSolver.java:40-41)
    LAUNCH_ANGLE_MAX = math.radians(48.5)
    # Hood range of 112 degrees maps to launch angle range
    # via (15/131) * (40/20) = 0.229 deg servo per deg launch
    SERVO_PER_LAUNCH_DEG = (15.0/131) * (40.0/20)
    LAUNCH_ANGLE_MIN = LAUNCH_ANGLE_MAX - math.radians((HOOD_MAX - HOOD_MIN) * SERVO_PER_LAUNCH_DEG)

    # Flywheel limits
    MAX_RPM = 4000
    RPM_PER_SEC_IN = 8.21238
    MAX_VELOCITY = MAX_RPM / RPM_PER_SEC_IN

    # Turret offset
    TURRET_OFFSET_Y = -2.559  # inches, from Common.java:69

    # Kinematics (from KinematicsSolver.java:45-59)
    GRAVITY = -386.0886  # in/s^2
    GOAL_HEIGHT = 40  # inches
    RIM_HEIGHT = 38.75
    BALL_RADIUS = 2.5
    WHEEL_RADIUS_PHYSICAL = 1.5
    COMPRESSION = 6/25.4
    WHEEL_RADIUS = WHEEL_RADIUS_PHYSICAL - COMPRESSION
    C = WHEEL_RADIUS + BALL_RADIUS  # 3.986 inches

    # Wheel position relative to turret
    WHEEL_X = 3.64584291339
    WHEEL_Y = 10.611220472440944

    # Goal center calculation (KinematicsSolver.java:34, 91-99)
    HALF_F = 141.5/2  # 70.75
    O_GOAL_X = 5
    O_GOAL_Y = -2.5

def launch_rad_to_servo(rad):
    """Convert launch radians to hood servo angle (Hood.java:55-58)"""
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    # launchRadiansToServoAngle: MIN + toDegrees(θ_launchMax - targetRadians) * scaleFactor
    return Constants.HOOD_MIN + (math.degrees(Constants.LAUNCH_ANGLE_MAX - rad)) * scale_factor

def servo_to_launch_rad(servo_angle):
    """Convert servo angle to launch radians (inverse of above)"""
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    launch_deg = max_deg - (servo_angle - Constants.HOOD_MIN) / scale_factor
    return math.radians(launch_deg)

def curve_hood_distance(d):
    """Hood curve fitting from distance (Hood.java:43-44)"""
    # 21.25528184671319 + 1.551657839403464*d - 0.00445309317429351*d^2
    val = 21.25528184671319 + 1.551657839403464 * d - 0.00445309317429351 * d * d
    return max(Constants.HOOD_MIN, min(Constants.HOOD_MAX, val))

def curve_rpm_distance(d):
    """Flywheel RPM curve fitting from distance (Flywheel.java:190)"""
    # 957.2952559300876 + 14.312109862671662*d
    val = 957.2952559300876 + 14.312109862671662 * d
    return max(0, min(Constants.MAX_RPM, val))

def calculate_shooting_params(robot_x, robot_y, robot_heading):
    """
    Calculate shooting parameters using physics-based approach
    similar to KinematicsSolver.java
    """
    # Goal position (for red alliance)
    G_x = Constants.HALF_F + (Constants.HALF_F - Constants.O_GOAL_X)
    G_y = Constants.HALF_F + Constants.HALF_F + Constants.O_GOAL_Y

    # Turret position
    turret_offset_x = Constants.TURRET_OFFSET_Y * math.cos(robot_heading)
    turret_offset_y = Constants.TURRET_OFFSET_Y * math.sin(robot_heading)
    turret_x = robot_x + turret_offset_x
    turret_y = robot_y + turret_offset_y

    # Distance and bearing to goal
    dx = G_x - turret_x
    dy = G_y - turret_y
    dist_to_goal = math.sqrt(dx*dx + dy*dy)
    bearing_to_goal = math.atan2(dy, dx)

    # Turret angle needed (relative to robot)
    turret_angle = bearing_to_goal - robot_heading
    turret_angle_deg = math.degrees(turret_angle)
    while turret_angle_deg > 180: turret_angle_deg -= 360
    while turret_angle_deg < -180: turret_angle_deg += 360

    # Launch point calculation
    # s0 is the wheel position relative to turret in launch direction
    # For stationary robot facing goal, launch point is offset from turret
    launch_offset_x = Constants.WHEEL_X

    # Horizontal distance from launch point to goal
    horizontal_dist = dist_to_goal - launch_offset_x

    # Vertical distance from launch point to goal
    launch_height = Constants.WHEEL_Y
    vertical_dist = Constants.GOAL_HEIGHT - launch_height

    # For projectile to reach (horizontal_dist, vertical_dist):
    # y = x*tan(θ) - (g*x^2)/(2*v^2*cos^2(θ))
    # Rearranging: v^2 = (g*x^2) / (2*cos^2(θ)*(x*tan(θ) - y))

    # We want to find θ that minimizes v while still reaching target
    # Check angles in [LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX]

    best_theta = None
    best_v = None
    min_v = float('inf')

    n_steps = 200
    for i in range(n_steps):
        theta = Constants.LAUNCH_ANGLE_MIN + \
                (Constants.LAUNCH_ANGLE_MAX - Constants.LAUNCH_ANGLE_MIN) * i / (n_steps - 1)

        tan_theta = math.tan(theta)
        cos_theta = math.cos(theta)

        # Must have positive horizontal velocity component
        if cos_theta <= 0.01:
            continue

        # Time to travel horizontal_dist: t = horizontal_dist / (v * cos_theta)
        # vertical position: y = launch_height + v*sin(theta)*t + 0.5*GRAVITY*t^2
        # At goal: GOAL_HEIGHT = launch_height + horizontal_dist*tan(theta) + 0.5*GRAVITY*t^2
        # Solving: t = horizontal_dist / (v * cos_theta)
        # GOAL_HEIGHT - launch_height - horizontal_dist*tan(theta) = 0.5*GRAVITY*(horizontal_dist/(v*cos_theta))^2

        # Let h = vertical_dist - horizontal_dist*tan(theta)
        # h = 0.5*GRAVITY*(horizontal_dist/(v*cos_theta))^2
        # h / (0.5*GRAVITY) = (horizontal_dist/(v*cos_theta))^2
        # sqrt(h / (0.5*GRAVITY)) = horizontal_dist/(v*cos_theta)
        # v = horizontal_dist / (cos_theta * sqrt(h / (0.5*GRAVITY)))

        h = vertical_dist - horizontal_dist * tan_theta

        # For valid solution, h and GRAVITY must have same sign (both negative)
        if h >= 0:
            continue

        # Calculate required velocity
        try:
            v_squared = (Constants.GRAVITY * horizontal_dist * horizontal_dist) / \
                       (2 * cos_theta * cos_theta * h)

            if v_squared > 0:
                v = math.sqrt(v_squared)

                if 0 < v <= Constants.MAX_VELOCITY:
                    if v < min_v:
                        min_v = v
                        best_theta = theta
                        best_v = v
        except:
            pass

    if best_theta is None:
        return turret_angle_deg, Constants.HOOD_MIN, 0, False

    hood_servo = launch_rad_to_servo(best_theta)
    rpm = best_v * Constants.RPM_PER_SEC_IN

    return turret_angle_deg, hood_servo, rpm, True

def main():
    print("=" * 130)
    print("TREND ANALYSIS: KinematicsSolver vs Curve Fitting")
    print("Robot is STATIONARY (vx=vy=angVel=0) on a 144x144 field")
    print(f"Goal position: ({Constants.GOAL_X}, {Constants.GOAL_Y}) - Center of far wall")
    print("=" * 130)
    print()
    print(f"{'Point':<10} | {'Dist':<6} | {'Turret Angle (deg)':<26} | {'Hood Servo Angle':<28} | {'Flywheel RPM':<26} |")
    print(f"{'(x,y)':<10} | {'(in)':<6} | {'Kinematics':<12} {'Curve':<12} | {'Kinematics':<13} {'Curve':<12} | {'Kinematics':<12} {'Curve':<11} |")
    print("-" * 130)

    # Test points across the field
    test_points = [
        # Far zone - long shots (80-120 inches)
        (30, 40), (72, 40), (114, 40),
        (30, 60), (72, 60), (114, 60),
        (30, 80), (72, 80), (114, 80),

        # Medium zone (40-70 inches)
        (40, 100), (72, 100), (104, 100),
        (50, 110), (72, 110), (94, 110),

        # Close zone (20-40 inches)
        (50, 120), (72, 120), (94, 120),
        (60, 125), (72, 125), (84, 125),

        # Edge cases
        (20, 80), (124, 80),
        (30, 100), (114, 100),
        (40, 130), (104, 130),
    ]

    results = []

    for x, y in test_points:
        # Calculate heading to face goal directly
        dx = Constants.GOAL_X - x
        dy = Constants.GOAL_Y - y
        heading = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)

        # Kinematics-based calculation
        kin_turret, kin_hood, kin_rpm, valid = calculate_shooting_params(x, y, heading)

        # Curve fitting calculation
        curve_hood = curve_hood_distance(distance)
        curve_rpm = curve_rpm_distance(distance)
        curve_turret = 0.0  # Assumes facing goal

        results.append({
            'x': x, 'y': y, 'dist': distance,
            'kin_turret': kin_turret, 'kin_hood': kin_hood, 'kin_rpm': kin_rpm, 'valid': valid,
            'curve_turret': curve_turret, 'curve_hood': curve_hood, 'curve_rpm': curve_rpm
        })

        valid_marker = "" if valid else " *"
        print(f"({x:3.0f},{y:3.0f})  | {distance:6.1f} | "
              f"{kin_turret:11.1f}  {curve_turret:11.1f} | "
              f"{kin_hood:12.1f}  {curve_hood:11.1f} | "
              f"{kin_rpm:11.1f}  {curve_rpm:10.1f} |{valid_marker}")

    print("-" * 130)
    print("\n* = Kinematics: No valid solution within launch angle/velocity constraints")

    # Analysis
    print("\n" + "=" * 130)
    print("ANALYSIS SUMMARY")
    print("=" * 130)
    print()
    print("CURVE FITTING EQUATIONS (from Java source):")
    print()
    print("  Hood Servo Angle (degrees):")
    print("    hood = 21.255 + 1.552*d - 0.00445*d^2")
    print("    where d = distance to goal in inches")
    print()
    print("  Flywheel RPM:")
    print("    rpm = 957.295 + 14.312*d")
    print("    where d = distance to goal in inches")
    print()
    print("MECHANICAL LIMITS:")
    max_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    print(f"  Hood Servo: [{Constants.HOOD_MIN}, {Constants.HOOD_MAX}] degrees")
    print(f"  Launch Angle: [{min_launch_deg:.1f}, {max_launch_deg:.1f}] degrees")
    print(f"  Flywheel: [0, {Constants.MAX_RPM}] RPM")
    print(f"  Max Velocity: {Constants.MAX_VELOCITY:.1f} in/s")
    print()
    print("TRENDS BY DISTANCE:")
    print()

    # Group by distance ranges
    ranges = [(0, 40), (40, 70), (70, 100), (100, 150)]
    for min_d, max_d in ranges:
        range_results = [r for r in results if min_d <= r['dist'] < max_d]
        if not range_results:
            continue

        valid_in_range = [r for r in range_results if r['valid']]
        if valid_in_range:
            avg_kin_hood = sum(r['kin_hood'] for r in valid_in_range) / len(valid_in_range)
            avg_kin_rpm = sum(r['kin_rpm'] for r in valid_in_range) / len(valid_in_range)
        else:
            avg_kin_hood = 0
            avg_kin_rpm = 0

        avg_curve_hood = sum(r['curve_hood'] for r in range_results) / len(range_results)
        avg_curve_rpm = sum(r['curve_rpm'] for r in range_results) / len(range_results)

        print(f"  Distance [{min_d}-{max_d}] inches:")
        print(f"    Kinematics:  Hood={avg_kin_hood:6.1f} deg, RPM={avg_kin_rpm:7.1f}")
        print(f"    Curve Fit:   Hood={avg_curve_hood:6.1f} deg, RPM={avg_curve_rpm:7.1f}")
        print()

    print("KEY OBSERVATIONS:")
    print()
    print("  1. HOOD ANGLE COMPARISON:")
    print()
    print("     Curve Fitting:")
    print("       - Parabolic: increases to peak at d=174 inches, then decreases")
    peak_d = 1.551657839403464 / (2 * 0.00445309317429351)
    peak_hood = curve_hood_distance(peak_d)
    print(f"       - Peak: {peak_hood:.1f} degrees at d={peak_d:.1f} inches")
    print("       - Hood approaches MIN (68) at very close distances")
    print()
    print("     Kinematics Solver:")
    print("       - Physics-based projectile motion")
    print("       - Closer shots: shallower angles (lower hood servo values)")
    print("       - Longer shots: steeper angles (higher hood servo values)")
    print("       - Constrained by Hood servo limits")
    print()
    print("  2. FLYWHEEL RPM COMPARISON:")
    print()
    print("     Curve Fitting:")
    print("       - Linear: +14.31 RPM per inch of distance")
    print("       - Base: 957 RPM at d=0")
    print(f"       - At d=144: {curve_rpm_distance(144):.0f} RPM")
    print()
    print("     Kinematics Solver:")
    print("       - Based on projectile energy requirements")
    print("       - Non-linear relationship")
    print("       - Very close shots may be impossible (rim interference)")
    print()
    print("  3. WHEN TO USE EACH METHOD:")
    print()
    print("     Use Curve Fitting when:")
    print("       - Need fast computation")
    print("       - Robot is stationary and known to work with fitted curves")
    print("       - Empirical testing validates the curves")
    print()
    print("     Use Kinematics Solver when:")
    print("       - Robot is moving (velocity compensation)")
    print("       - Shooting from unconventional positions")
    print("       - Need to verify rim clearance")
    print("       - Maximum accuracy required")
    print()
    print("  4. STATIONARY ROBOT NOTES:")
    print()
    print("     For stationary robot facing goal:")
    print("       - Turret angle = 0 (already aligned)")
    print("       - v_turret = 0 (no motion compensation)")
    print("       - alpha_launch = 0 (no offset needed)")
    print("       - Pure projectile motion problem")
    print()
    print("     The two methods SHOULD produce similar results if curve was fit")
    print("     using stationary data. Differences indicate:")
    print("       - Curve was fit with different assumptions")
    print("       - Curve accounts for real-world factors not in kinematics")
    print("       - Mechanical/systematic errors in the system")
    print("=" * 130)

if __name__ == "__main__":
    main()
