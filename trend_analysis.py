#!/usr/bin/env python3
"""
Trend Analysis: KinematicsSolver vs Curve Fitting
Robot is stationary (vx=vy=angVel=0) on a 144x144 field

UPDATED CONSTANTS (latest branch):
- Hood.java: MIN=68, MAX=160, PHYSICAL_MAX=195
- KinematicsSolver.java: θ_launchMax=68°, scale=(13/127)*(60/20)
- Flywheel.java: RPM_PER_SEC_IN=8.17067
"""

import math

class Constants:
    """Physical constants from latest Java code"""
    # Goal position
    GOAL_X = 72
    GOAL_Y = 144

    # Hood servo limits (Hood.java:24-27)
    HOOD_MIN = 80
    HOOD_MAX = 160  # Updated from 180

    # Launch angle limits (KinematicsSolver.java:39-40)
    # θ_launchMax = toRadians(68) - Updated from 48.5
    LAUNCH_ANGLE_MAX = math.radians(68)
    # Scale: (13.0/127) * (60.0/20) = ~0.3071 - Updated from (15/131)*(40/20)
    SERVO_PER_LAUNCH_DEG = (13.0/127) * (60.0/20)
    # θ_launchMin = 68° - (160-68)*0.3071 = 68° - 28.3° = 39.7°
    LAUNCH_ANGLE_MIN = LAUNCH_ANGLE_MAX - math.radians((HOOD_MAX - HOOD_MIN) * SERVO_PER_LAUNCH_DEG)

    # Flywheel limits (Flywheel.java:39,52)
    RPM_PER_SEC_IN = 8.17067  # Updated from 8.21238
    MAX_RPM = 4000
    MAX_VELOCITY = MAX_RPM / RPM_PER_SEC_IN

    # Turret offset (Common.java:69)
    TURRET_OFFSET_Y = -2.559

    # Kinematics constants (KinematicsSolver.java:44-58)
    GRAVITY = -386.0886
    GOAL_HEIGHT = 40
    RIM_HEIGHT = 38.75
    RIM_CLEARANCE = 0.75
    BALL_RADIUS = 2.5
    WHEEL_PHYSICAL_RADIUS = 1.5
    COMPRESSION = 6/25.4
    WHEEL_EFFECTIVE_RADIUS = WHEEL_PHYSICAL_RADIUS - COMPRESSION
    C = WHEEL_EFFECTIVE_RADIUS + BALL_RADIUS

    # Wheel position (KinematicsSolver.java:57)
    WHEEL_X = 3.64584291339
    WHEEL_Y = 10.611220472440944

    # Goal calculation (KinematicsSolver.java:33,53,88-99)
    HALF_F = 141.5 / 2
    O_GOAL_X = 5
    O_GOAL_Y = -2.5

def launch_rad_to_servo_angle(rad):
    """
    Convert launch radians to hood servo angle (Hood.java:61-65)

    Hood.launchRadiansToServoAngle:
        scaleFactor = (MAX - MIN) / (toDegrees(θ_launchMax) - toDegrees(θ_launchMin))
        return MIN + toDegrees(θ_launchMax - targetRadians) * scaleFactor

    With current values:
    - θ_launchMax = 68° → Hood.MIN = 68° (steepest shot)
    - θ_launchMin = 39.7° → Hood.MAX = 160° (shallowest shot)
    """
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    return Constants.HOOD_MIN + (max_deg - math.degrees(rad)) * scale_factor

def servo_angle_to_launch_rad(servo_angle):
    """Convert servo angle back to launch radians"""
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    launch_deg = max_deg - (servo_angle - Constants.HOOD_MIN) / scale_factor
    return math.radians(launch_deg)

def curve_hood_from_distance(distance):
    """Hood curve fitting (Hood.java:50)"""
    # 21.255 + 1.552*d - 0.00445*d^2
    result = 21.25528184671319 + 1.551657839403464 * distance - 0.00445309317429351 * distance * distance
    return max(Constants.HOOD_MIN, min(Constants.HOOD_MAX, result))

def curve_flywheel_from_distance(distance):
    """Flywheel RPM curve fitting (Flywheel.java:189)"""
    # 957.295 + 14.312*d
    result = 957.2952559300876 + 14.312109862671662 * distance
    return max(0, min(Constants.MAX_RPM, result))

class KinematicsSimulator:
    """Simulates the KinematicsSolver algorithm"""

    def __init__(self):
        self.s0 = [Constants.WHEEL_X, Constants.WHEEL_Y]

    def theta_3pt(self, target_x, target_y, goal_x, goal_y):
        """3-point trajectory angle (KinematicsSolver.java:238-249)"""
        s0_x = self.s0[0]
        s0_y = self.s0[1]

        dx = goal_x - s0_x
        dy = goal_y - s0_y
        tx = target_x - s0_x
        ty = target_y - s0_y

        if abs(dx * tx * (dx - tx)) < 0.001:
            return 0

        numerator = dx * dx * ty - tx * tx * dy
        denominator = dx * tx * (dx - tx)

        return math.atan(numerator / denominator)

    def v1(self, cos_theta, tan_theta, goal_x, goal_y):
        """Velocity from angle (KinematicsSolver.java:225-228)"""
        s0_x = self.s0[0]
        s0_y = self.s0[1]

        dx = goal_x - s0_x
        discriminant = goal_y - s0_y - dx * tan_theta

        if discriminant >= 0:
            return float('inf')

        val = (2.0 / Constants.GRAVITY) * discriminant
        if val <= 0:
            return float('inf')

        return dx / (cos_theta * math.sqrt(val))

    def calculate_target(self, distance_to_goal):
        """
        Simplified calculateTarget_v_θ_α (KinematicsSolver.java:258-313)
        For stationary robot
        """
        goal_x = distance_to_goal
        goal_y = Constants.GOAL_HEIGHT

        # Rim position (simplified)
        rim_dist_from_goal = 15
        rim_x = distance_to_goal - rim_dist_from_goal
        rim_y = Constants.RIM_HEIGHT

        theta_launch = (Constants.LAUNCH_ANGLE_MIN + Constants.LAUNCH_ANGLE_MAX) / 2
        v_launch = 200

        # Iterate to find valid solution
        for iteration in range(5):
            # Angle to clear rim
            rim_clearance_y = rim_y + Constants.BALL_RADIUS + Constants.RIM_CLEARANCE
            theta_to_rim = self.theta_3pt(rim_x, rim_clearance_y, goal_x, goal_y)

            # Clamp to valid range
            theta_to_rim = max(Constants.LAUNCH_ANGLE_MIN, min(Constants.LAUNCH_ANGLE_MAX, theta_to_rim))

            cos_theta = math.cos(theta_to_rim)
            sin_theta = math.sin(theta_to_rim)

            if cos_theta <= 0.01:
                continue

            tan_theta = sin_theta / cos_theta
            v_needed = self.v1(cos_theta, tan_theta, goal_x, goal_y)

            if 0 < v_needed <= Constants.MAX_VELOCITY:
                theta_launch = theta_to_rim
                v_launch = v_needed
            else:
                # Try shallower angle
                theta_to_rim = (theta_to_rim + Constants.LAUNCH_ANGLE_MAX) / 2
                cos_theta = math.cos(theta_to_rim)
                if cos_theta > 0.01:
                    tan_theta = math.sin(theta_to_rim) / cos_theta
                    v_needed = self.v1(cos_theta, tan_theta, goal_x, goal_y)
                    if 0 < v_needed <= Constants.MAX_VELOCITY:
                        theta_launch = theta_to_rim
                        v_launch = v_needed

        # Validate
        if v_launch <= 0 or v_launch > Constants.MAX_VELOCITY:
            return None, None, False

        hood_servo = launch_rad_to_servo_angle(theta_launch)
        rpm = v_launch * Constants.RPM_PER_SEC_IN

        return hood_servo, rpm, True

    def solve(self, robot_x, robot_y, robot_heading):
        """Calculate shooting parameters"""
        # Goal position (red alliance)
        i = 1
        G_x = Constants.HALF_F + (Constants.HALF_F - Constants.O_GOAL_X) * i
        G_y = Constants.HALF_F + Constants.HALF_F + Constants.O_GOAL_Y

        # Turret position
        turret_x = robot_x + Constants.TURRET_OFFSET_Y * math.cos(robot_heading)
        turret_y = robot_y + Constants.TURRET_OFFSET_Y * math.sin(robot_heading)

        # Distance to goal
        dx = G_x - turret_x
        dy = G_y - turret_y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)

        # Turret angle
        turret_angle = angle_to_goal - robot_heading
        turret_angle_deg = math.degrees(turret_angle)
        while turret_angle_deg > 180: turret_angle_deg -= 360
        while turret_angle_deg < -180: turret_angle_deg += 360

        # Calculate kinematics
        hood_servo, rpm, valid = self.calculate_target(distance_to_goal)

        return turret_angle_deg, hood_servo, rpm, valid, distance_to_goal

def print_header():
    max_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)

    print("=" * 130)
    print("TREND ANALYSIS: KinematicsSolver vs Curve Fitting")
    print("Robot is STATIONARY (vx=vy=angVel=0) on a 144x144 field")
    print(f"Goal position: ({Constants.GOAL_X}, {Constants.GOAL_Y}) - Center of far wall (Red Alliance)")
    print("=" * 130)
    print()
    print("CURRENT CONSTANTS:")
    print(f"  Hood.java: MIN={Constants.HOOD_MIN}, MAX={Constants.HOOD_MAX}")
    print(f"  KinematicsSolver.java:")
    print(f"    theta_launchMax = {max_launch_deg} deg -> Hood.MIN = {Constants.HOOD_MIN}")
    print(f"    theta_launchMin = {min_launch_deg:.1f} deg -> Hood.MAX = {Constants.HOOD_MAX}")
    print(f"    Scale factor = {Constants.SERVO_PER_LAUNCH_DEG:.4f}")
    print(f"  Flywheel.java: RPM_PER_SEC_IN = {Constants.RPM_PER_SEC_IN}")
    print()
    print(f"{'Point':<10} | {'Dist':<6} | {'TURRET (deg)':<24} | {'HOOD (servo)':<26} | {'FLYWHEEL (RPM)':<24} |")
    print(f"{'(x,y)':<10} | {'(in)':<6} | {'Kinem.':<10} {'Curve':<10} | {'Kinem.':<11} {'Curve':<11} | {'Kinem.':<11} {'Curve':<10} |")
    print("-" * 130)

def main():
    print_header()

    # Define test points across the field
    test_points = [
        (30, 40), (72, 40), (114, 40),
        (30, 60), (72, 60), (114, 60),
        (30, 80), (72, 80), (114, 80),
        (40, 100), (72, 100), (104, 100),
        (50, 110), (72, 110), (94, 110),
        (50, 120), (72, 120), (94, 120),
        (60, 125), (72, 125), (84, 125),
        (20, 80), (124, 80),
        (30, 100), (114, 100),
    ]

    solver = KinematicsSimulator()
    results = []

    for x, y in test_points:
        # Calculate heading to face the goal
        dx = Constants.GOAL_X - x
        dy = Constants.GOAL_Y - y
        heading = math.atan2(dy, dx)

        # Kinematics solver results
        kin_turret, kin_hood, kin_flywheel, valid, distance = solver.solve(x, y, heading)

        # Curve fitting results
        curve_hood = curve_hood_from_distance(distance)
        curve_flywheel = curve_flywheel_from_distance(distance)
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

    print("-" * 130)
    print("\n* = No valid kinematics solution found (distance may be outside achievable range)")

    # Calculate trend statistics
    valid_results = [r for r in results if r['valid']]

    max_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)

    print("\n" + "=" * 130)
    print("CURVE FITTING EQUATIONS (from Hood.java:50 and Flywheel.java:189):")
    print()
    print("  Hood servo angle  = 21.255 + 1.552*d - 0.00445*d^2")
    print("  Flywheel RPM      = 957.295 + 14.312*d")
    print("  where d = distance to goal in inches")
    print()
    print("LAUNCH ANGLE LIMITS (from Hood.java:61-65 and KinematicsSolver.java:39-40):")
    print(f"  LAUNCH_ANGLE_MAX = {max_launch_deg} degrees (maps to Hood.MIN = {Constants.HOOD_MIN} degrees)")
    print(f"  LAUNCH_ANGLE_MIN = {min_launch_deg:.1f} degrees (maps to Hood.MAX = {Constants.HOOD_MAX} degrees)")
    print()
    print("MECHANICAL MAPPING:")
    print(f"  Hood servo range: [{Constants.HOOD_MIN}, {Constants.HOOD_MAX}] degrees")
    print(f"  Launch angle range: [{min_launch_deg:.1f}, {max_launch_deg:.1f}] degrees")
    scale = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_launch_deg - min_launch_deg)
    print(f"  Scale factor: {scale:.2f} servo degrees per launch degree")
    print()
    print("INVERSE RELATIONSHIP:")
    print(f"  - Higher launch angle (steeper shot, up to {max_launch_deg} deg) -> Lower servo value (MIN={Constants.HOOD_MIN})")
    print(f"  - Lower launch angle (flatter shot, down to {min_launch_deg:.1f} deg) -> Higher servo value (MAX={Constants.HOOD_MAX})")
    print()
    print("=" * 130)
    print("KEY TREND OBSERVATIONS:")
    print()
    print("  1. HOOD ANGLE TREND:")
    print()
    print("     CURVE FITTING (Polynomial):")
    print("       - Parabolic relationship with distance")
    print("       - Hood angle INCREASES with distance up to peak")
    peak_dist = 1.551657839403464 / (2 * 0.00445309317429351)
    peak_hood = 21.25528184671319 + 1.551657839403464 * peak_dist - 0.00445309317429351 * peak_dist * peak_dist
    print(f"       - Peak at d={peak_dist:.1f} in: hood={peak_hood:.1f}°")
    print()
    print("     KINEMATICS SOLVER (Physics-based):")
    print("       - Closer distances: shallower angles (servo closer to MAX)")
    print("       - Longer distances: steeper angles (servo closer to MIN)")
    print("       - Constrained by launch angle limits")
    print()
    print("  2. FLYWHEEL RPM TREND:")
    print()
    print("     CURVE FITTING:")
    print("       - Linear: +14.31 RPM per inch")
    print(f"       - Range: {curve_flywheel_from_distance(0):.0f} to {curve_flywheel_from_distance(144):.0f} RPM")
    print()
    print("     KINEMATICS SOLVER:")
    print("       - Physics-based, non-linear")
    print("       - Longer shots need more energy")
    print()
    print("  3. VALID RANGES:")
    print(f"       - Hood servo: [{Constants.HOOD_MIN}, {Constants.HOOD_MAX}] degrees")
    print(f"       - Flywheel: [0, {Constants.MAX_RPM}] RPM")
    print(f"       - Launch angle: [{min_launch_deg:.1f}, {max_launch_deg:.1f}] degrees")
    print("=" * 130)

if __name__ == "__main__":
    main()
