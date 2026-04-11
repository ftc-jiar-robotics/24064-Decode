#!/usr/bin/env python3
"""
Trend Analysis: KinematicsSolver vs Curve Fitting
Robot is stationary (vx=vy=angVel=0) on a 144x144 field
Goal is at the center of the far wall
"""

import math

class Constants:
    """Physical constants from the Java code"""
    # Goal position
    GOAL_X = 72
    GOAL_Y = 144

    # Hood constants
    HOOD_MIN = 68
    HOOD_MAX = 180

    # Launch angle limits (from KinematicsSolver)
    # Hood range: 112 degrees (MAX - MIN)
    # Maps to launch angle range via mechanical linkage
    LAUNCH_ANGLE_MAX = math.radians(48.5)  # 48.5 degrees in radians
    # MIN = MAX - toRadians((MAX-MIN) * (15/131) * (40/20))
    LAUNCH_ANGLE_MIN = LAUNCH_ANGLE_MAX - math.radians((180 - 68) * (15.0/131) * (40.0/20))

    # Flywheel constants
    RPM_PER_SEC_IN = 8.21238
    MAX_RPM = 4000
    MAX_VELOCITY = MAX_RPM / RPM_PER_SEC_IN  # in/s

    # Turret offset
    TURRET_OFFSET_Y = -2.559

    # Kinematics constants
    GRAVITY = -386.0886  # Gravity in in/s^2 (negative = down)
    GOAL_HEIGHT = 40  # Goal height in inches
    RIM_HEIGHT = 38.75  # Rim height
    BALL_RADIUS = 2.5  # Ball radius
    WHEEL_PHYSICAL_RADIUS = 1.5
    COMPRESSION = 6/25.4  # Compression in inches
    WHEEL_EFFECTIVE_RADIUS = WHEEL_PHYSICAL_RADIUS - COMPRESSION
    C = WHEEL_EFFECTIVE_RADIUS + BALL_RADIUS  # Distance from wheel center to ball center

    # Wheel position relative to turret (constant)
    WHEEL_X = 3.64584291339
    WHEEL_Y = 10.611220472440944

def launch_rad_to_servo_angle(rad):
    """Convert launch radians to servo angle using linear mapping from Hood.java"""
    # From Hood.launchRadiansToServoAngle():
    # scaleFactor = (MAX - MIN) / (maxLaunchDeg - minLaunchDeg)
    # return MIN + (maxLaunchRad - targetRad) * scaleFactor * (180/pi)
    max_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_launch_deg - min_launch_deg)
    return Constants.HOOD_MIN + (max_launch_deg - math.degrees(rad)) * scale_factor

def servo_angle_to_launch_rad(servo_angle):
    """Convert servo angle back to launch radians"""
    max_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_launch_deg - min_launch_deg)
    return math.radians(max_launch_deg - (servo_angle - Constants.HOOD_MIN) / scale_factor)

def curve_hood_from_distance(distance):
    """Curve fitting: hood servo angle from distance (Hood.java:44)"""
    # 21.255 + 1.552*distance - 0.00445*distance^2
    result = 21.25528184671319 + 1.551657839403464 * distance - 0.00445309317429351 * distance * distance
    return max(Constants.HOOD_MIN, min(Constants.HOOD_MAX, result))

def curve_flywheel_from_distance(distance):
    """Curve fitting: flywheel RPM from distance (Flywheel.java:190)"""
    # 957.295 + 14.312*distance
    result = 957.2952559300876 + 14.312109862671662 * distance
    return max(0, min(Constants.MAX_RPM, result))

def curve_hood_from_rpm(rpm):
    """Alternative curve fitting: hood angle from RPM (Hood.java:49-50)"""
    # -87.519 + 0.137*rpm - 0.0000189*rpm^2
    return -87.51927299612511 + 0.1366987334209007 * rpm - 1.888596858075093e-05 * rpm * rpm

def solve_projectile_motion(distance, target_height, y_launch):
    """
    Solve for launch angle given distance and heights.
    Returns (angle_rad, velocity) or (None, None) if no solution.

    For projectile motion from (0, y0) to (d, y_target):
    y = y0 + x*tan(theta) + (g*x^2)/(2*v^2*cos^2(theta))

    Let u = tan(theta), then cos^2(theta) = 1/(1+u^2)
    y_target = y0 + d*u + (g*d^2*(1+u^2))/(2*v^2)

    Rearranging:
    v^2 = (g*d^2*(1+u^2)) / (2*(y_target - y0 - d*u))

    We want to find angle in [LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX]
    that gives valid velocity in [0, MAX_VELOCITY]
    """
    delta_y = target_height - y_launch
    best_solution = None
    min_velocity = float('inf')

    # Try launch angles across valid range
    n_angles = 100
    for i in range(n_angles):
        theta = Constants.LAUNCH_ANGLE_MIN + \
                (Constants.LAUNCH_ANGLE_MAX - Constants.LAUNCH_ANGLE_MIN) * i / (n_angles - 1)

        tan_theta = math.tan(theta)

        # Required velocity squared
        # From: y = y0 + d*tan(theta) + (g*d^2)/(2*v^2*cos^2(theta))
        # v^2 = (g*d^2) / (2*cos^2(theta)*(y - y0 - d*tan(theta)))
        # Note: GRAVITY is negative, so we need positive discriminant

        cos_theta = math.cos(theta)
        if abs(cos_theta) < 0.001:
            continue

        # Discriminant check: can we reach this height?
        # For upward launch (theta > 0), we need enough vertical velocity
        if theta > 0:
            # Time to reach target: t = d / (v * cos(theta))
            # y = y0 + v*sin(theta)*t + 0.5*g*t^2
            # y = y0 + d*tan(theta) + 0.5*g*d^2/(v^2*cos^2(theta))
            # y - y0 - d*tan(theta) = 0.5*g*d^2/(v^2*cos^2(theta))
            # For this to be achievable, y - y0 - d*tan(theta) must have same sign as g

            discriminant = delta_y - distance * tan_theta
            if discriminant >= 0:
                # Need to check if we can reach this
                continue

            # Calculate required velocity
            v_squared = (Constants.GRAVITY * distance * distance) / \
                       (2 * cos_theta * cos_theta * discriminant)

            if v_squared > 0:
                v = math.sqrt(v_squared)
                if 0 < v <= Constants.MAX_VELOCITY:
                    if v < min_velocity:
                        min_velocity = v
                        best_solution = (theta, v)
        else:
            # Downward launch - generally not useful for this application
            pass

    return best_solution if best_solution else (None, None)

def solve_kinematics(robot_x, robot_y, heading):
    """
    Calculate shooting parameters for a stationary robot.
    Returns: (turret_angle_deg, hood_servo_angle, flywheel_rpm, valid)
    """
    # Goal is at (72, 144) for red alliance (far wall, center)
    goal_x = Constants.GOAL_X
    goal_y = Constants.GOAL_Y

    # Turret position
    turret_x = robot_x + Constants.TURRET_OFFSET_Y * math.cos(heading)
    turret_y = robot_y + Constants.TURRET_OFFSET_Y * math.sin(heading)

    # Distance and angle to goal
    dx = goal_x - turret_x
    dy = goal_y - turret_y
    distance_to_goal = math.sqrt(dx*dx + dy*dy)
    angle_to_goal = math.atan2(dy, dx)

    # Turret angle relative to robot heading
    turret_angle = angle_to_goal - heading
    turret_angle_deg = math.degrees(turret_angle)
    # Normalize to [-180, 180]
    while turret_angle_deg > 180:
        turret_angle_deg -= 360
    while turret_angle_deg < -180:
        turret_angle_deg += 360

    # Launch point position
    # s0.x = s_wheel.x = horizontal offset from turret
    launch_horizontal_dist = distance_to_goal - Constants.WHEEL_X
    launch_vertical_offset = Constants.WHEEL_Y

    # Solve projectile motion
    theta_launch, v_launch = solve_projectile_motion(
        launch_horizontal_dist,
        Constants.GOAL_HEIGHT,
        launch_vertical_offset
    )

    if theta_launch is None or v_launch is None:
        return turret_angle_deg, Constants.HOOD_MIN, 0, False

    # Convert to outputs
    hood_servo = launch_rad_to_servo_angle(theta_launch)
    flywheel_rpm = v_launch * Constants.RPM_PER_SEC_IN

    return turret_angle_deg, hood_servo, flywheel_rpm, True

def print_header():
    print("=" * 125)
    print("TREND ANALYSIS: KinematicsSolver vs Curve Fitting")
    print("Robot is STATIONARY (vx=vy=angVel=0) on a 144x144 field")
    print(f"Goal position: ({Constants.GOAL_X}, {Constants.GOAL_Y}) - Center of far wall (Red Alliance)")
    print("=" * 125)
    print()
    print(f"{'Point':<10} | {'Dist':<6} | {'TURRET (deg)':<24} | {'HOOD (servo)':<26} | {'FLYWHEEL (RPM)':<24} |")
    print(f"{'(x,y)':<10} | {'(in)':<6} | {'Kinem.':<10} {'Curve':<10} | {'Kinem.':<11} {'Curve':<11} | {'Kinem.':<11} {'Curve':<10} |")
    print("-" * 125)

def main():
    print_header()

    # Define test points across the field
    test_points = [
        # Far zone - long shots (80-120 inches)
        (30, 40), (72, 40), (114, 40),      # Back row
        (30, 60), (72, 60), (114, 60),      # Middle-back
        (30, 80), (72, 80), (114, 80),      # Middle (70-80 inches)

        # Medium zone (40-70 inches)
        (40, 100), (72, 100), (104, 100),   # Front-middle
        (50, 110), (72, 110), (94, 110),    # Closer

        # Close zone (20-40 inches)
        (50, 120), (72, 120), (94, 120),    # Close to goal
        (60, 125), (72, 125), (84, 125),    # Very close

        # Edge cases
        (20, 80), (124, 80),                # Far sides
        (30, 100), (114, 100),              # Side angles
        (40, 130), (104, 130),              # Extreme close sides
    ]

    # Track trends
    results = []

    for x, y in test_points:
        # Calculate heading to face the goal
        dx = Constants.GOAL_X - x
        dy = Constants.GOAL_Y - y
        heading = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)

        # Kinematics solver results
        kin_turret, kin_hood, kin_flywheel, valid = solve_kinematics(x, y, heading)

        # Curve fitting results
        curve_hood = curve_hood_from_distance(distance)
        curve_flywheel = curve_flywheel_from_distance(distance)

        # For curve fitting, assume turret is aligned to goal (0 offset)
        curve_turret = 0.0

        # Store results
        results.append({
            'x': x, 'y': y, 'distance': distance,
            'kin_turret': kin_turret, 'kin_hood': kin_hood, 'kin_flywheel': kin_flywheel, 'valid': valid,
            'curve_turret': curve_turret, 'curve_hood': curve_hood, 'curve_flywheel': curve_flywheel
        })

        # Print formatted row
        valid_str = "" if valid else "*"
        print(f"({x:3.0f},{y:3.0f})  | {distance:6.1f} | "
              f"{kin_turret:9.1f}  {curve_turret:9.1f} | "
              f"{kin_hood:10.1f}  {curve_hood:10.1f} | "
              f"{kin_flywheel:10.1f}  {curve_flywheel:9.1f} |{valid_str}")

    print("-" * 125)
    print("\n* = No valid kinematics solution found (distance may be outside achievable range)")

    # Calculate trend statistics
    valid_results = [r for r in results if r['valid']]

    print("\n" + "=" * 125)
    print("CURVE FITTING EQUATIONS (from Hood.java and Flywheel.java):")
    print()
    print("  Hood servo angle  = 21.255 + 1.552*d - 0.00445*d^2")
    print("  Flywheel RPM      = 957.295 + 14.312*d")
    print("  where d = distance to goal in inches")
    print()
    print("LAUNCH ANGLE LIMITS (from Hood.java:55-58):")
    max_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    print(f"  LAUNCH_ANGLE_MAX = {max_launch_deg:.1f} degrees (maps to Hood.MIN = {Constants.HOOD_MIN} degrees)")
    print(f"  LAUNCH_ANGLE_MIN = {min_launch_deg:.1f} degrees (maps to Hood.MAX = {Constants.HOOD_MAX} degrees)")
    print()
    print("MECHANICAL MAPPING:")
    print(f"  Hood servo range: [{Constants.HOOD_MIN}, {Constants.HOOD_MAX}] degrees")
    print(f"  Launch angle range: [{min_launch_deg:.1f}, {max_launch_deg:.1f}] degrees")
    print(f"  Scale factor: {112/(max_launch_deg-min_launch_deg):.2f} servo degrees per launch degree")
    print()
    print("=" * 125)
    print("KEY TREND OBSERVATIONS:")
    print()
    print("  1. HOOD ANGLE TREND:")
    print()
    print("     CURVE FITTING (Polynomial):")
    print("       - Parabolic relationship with distance")
    print("       - Hood angle INCREASES with distance up to peak")
    print("       - Peak occurs at d = -b/(2a) = 1.552/(2*0.00445) = 174.4 inches")
    print("       - At peak: hood angle = 21.255 + 1.552(174.4) - 0.00445(174.4)^2")
    peak_dist = 1.551657839403464 / (2 * 0.00445309317429351)
    peak_hood = 21.25528184671319 + 1.551657839403464 * peak_dist - 0.00445309317429351 * peak_dist * peak_dist
    print(f"         Peak hood angle = {peak_hood:.1f} degrees at d = {peak_dist:.1f} inches")
    print("       - Beyond peak, hood angle DECREASES (unphysical - curve limited by Hood.MAX)")
    print()
    print("     KINEMATICS SOLVER (Physics-based):")
    print("       - Based on projectile motion equations")
    print("       - At closer distances: shallower launch angles")
    print("       - At farther distances: steeper launch angles (up to limit)")
    print("       - Constrained by Hood servo limits [68, 180]")
    print()
    print("  2. FLYWHEEL RPM TREND:")
    print()
    print("     CURVE FITTING (Linear):")
    print("       - Linear increase: +14.31 RPM per inch of distance")
    print(f"       - Intercept: 957 RPM (idle/minimum speed)")
    print(f"       - At d=0: {curve_flywheel_from_distance(0):.1f} RPM")
    print(f"       - At d=50: {curve_flywheel_from_distance(50):.1f} RPM")
    print(f"       - At d=100: {curve_flywheel_from_distance(100):.1f} RPM")
    print(f"       - At d=144 (max): {curve_flywheel_from_distance(144):.1f} RPM")
    print()
    print("     KINEMATICS SOLVER (Physics-based):")
    print("       - Required velocity based on projectile energy")
    print("       - Non-linear relationship (quadratic in velocity)")
    print("       - Closer shots need less energy than curve predicts")
    print("       - Longer shots need significantly more RPM")
    print()
    print("  3. TURRET ANGLE:")
    print()
    print("     CURVE FITTING:")
    print("       - Assumes robot is ALWAYS facing goal (0 degrees)")
    print("       - No compensation for robot orientation")
    print()
    print("     KINEMATICS SOLVER:")
    print("       - Calculates actual angle from robot pose to goal")
    print("       - Accounts for robot heading vs goal bearing")
    print("       - Returns angle needed to align turret to goal")
    print()
    print("  4. STATIONARY ROBOT SIMPLIFICATION:")
    print()
    print("     When robot is stationary (vx=vy=angVel=0):")
    print("       - v_turret = 0 (no translational velocity)")
    print("       - v_relToGoal = 0 (no relative velocity)")
    print("       - alpha_launch (turret offset) = 0")
    print("       - Simplifies to pure projectile motion problem")
    print()
    print("  5. VALID RANGES:")
    print(f"       - Hood servo: [{Constants.HOOD_MIN}, {Constants.HOOD_MAX}] degrees")
    print(f"       - Flywheel: [0, {Constants.MAX_RPM}] RPM")
    print(f"       - Launch angle: [{min_launch_deg:.1f}, {max_launch_deg:.1f}] degrees")
    print(f"       - Max velocity: {Constants.MAX_VELOCITY:.1f} in/s ({Constants.MAX_RPM:.0f} RPM)")
    print()
    print("  6. DISCREPANCY ANALYSIS:")
    print()
    print("     The kinematics solver and curve fitting often produce DIFFERENT values")
    print("     because they use different approaches:")
    print()
    print("     Curve Fitting:")
    print("       - Empirical: based on measured data points")
    print("       - Simple polynomials for fast computation")
    print("       - Does NOT account for robot velocity")
    print("       - Hood curve is parabolic (peaks then decreases)")
    print()
    print("     Kinematics Solver:")
    print("       - Physics-based projectile motion model")
    print("       - Accounts for 3D trajectory, rim clearance, compression")
    print("       - Full state includes robot velocity compensation")
    print("       - Iterative solver finds valid (v, theta, alpha) triplets")
    print()
    print("     For STATIONARY robot, both should converge IF the curve was fit")
    print("     using stationary data. The difference indicates the curve")
    print("     may have been fit with different assumptions or constraints.")
    print("=" * 125)

if __name__ == "__main__":
    main()
