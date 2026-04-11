#!/usr/bin/env python3
"""
CORRECT Trend Analysis: KinematicsSolver vs Curve Fitting
Robot is stationary (vx=vy=angVel=0) on a 144x144 field

This version properly implements the KinematicsSolver algorithm from Java:
- Uses θ_3pt() to calculate angle for rim clearance
- Uses v1() to compute velocity from angle
- Follows the iterative solver in calculateTarget_v_θ_α()
"""

import math

class Constants:
    """Physical and mechanical constants from the Java code"""
    # Field
    GOAL_X = 72
    GOAL_Y = 144

    # Hood servo limits (Hood.java:20-22)
    HOOD_MIN = 68
    HOOD_MAX = 180

    # Launch angle limits (KinematicsSolver.java:40-41)
    LAUNCH_ANGLE_MAX = math.radians(48.5)  # θ_launchMax
    # Hood range 112 degrees maps via (15/131)*(40/20) = 0.229
    SERVO_PER_LAUNCH_DEG = (15.0/131) * (40.0/20)
    LAUNCH_ANGLE_MIN = LAUNCH_ANGLE_MAX - math.radians((HOOD_MAX - HOOD_MIN) * SERVO_PER_LAUNCH_DEG)
    # LAUNCH_ANGLE_MIN ≈ 22.9 degrees

    # Flywheel limits
    MAX_RPM = 4000
    RPM_PER_SEC_IN = 8.21238  # (Flywheel.java:39)
    MAX_VELOCITY = MAX_RPM / RPM_PER_SEC_IN  # ≈ 487 in/s

    # Turret offset (Common.java:69)
    TURRET_OFFSET_Y = -2.559

    # Kinematics (KinematicsSolver.java:45-59)
    GRAVITY = -386.0886  # a_G in in/s^2
    GOAL_HEIGHT = 40  # y_goal
    RIM_HEIGHT = 38.75  # y_rim
    RIM_CLEARANCE = 0.75  # r_rimClearance
    BALL_RADIUS = 2.5  # r_ball
    WHEEL_RADIUS_PHYSICAL = 1.5
    COMPRESSION = 6/25.4  # r_compression
    WHEEL_RADIUS = WHEEL_RADIUS_PHYSICAL - COMPRESSION
    C = WHEEL_RADIUS + BALL_RADIUS  # Distance from wheel center to ball center

    # Wheel position relative to turret (KinematicsSolver.java:58)
    WHEEL_X = 3.64584291339
    WHEEL_Y = 10.611220472440944

    # Goal calculation (KinematicsSolver.java:34, 91-99)
    HALF_F = 141.5 / 2  # 70.75
    O_GOAL_X = 5
    O_GOAL_Y = -2.5

    # Rim calculation (KinematicsSolver.java:94-99)
    A_RIM_X = 47.42197
    A_RIM_Y = 70.04449
    B_RIM_X = 15.85266
    B_RIM_Y = -21.85619

def launch_rad_to_servo(rad):
    """
    Convert launch radians to hood servo angle (Hood.java:55-58)
    Hood.MIN (68) corresponds to θ_launchMax (48.5°)
    Hood.MAX (180) corresponds to θ_launchMin (22.9°)
    """
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    # launchRadiansToServoAngle: MIN + toDegrees(θ_launchMax - targetRadians) * scaleFactor
    return Constants.HOOD_MIN + (math.degrees(Constants.LAUNCH_ANGLE_MAX - rad)) * scale_factor

def servo_to_launch_rad(servo_angle):
    """Convert servo angle to launch radians (inverse)"""
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    launch_deg = max_deg - (servo_angle - Constants.HOOD_MIN) / scale_factor
    return math.radians(launch_deg)

def curve_hood_distance(d):
    """Hood curve fitting (Hood.java:43-44)"""
    val = 21.25528184671319 + 1.551657839403464 * d - 0.00445309317429351 * d * d
    return max(Constants.HOOD_MIN, min(Constants.HOOD_MAX, val))

def curve_rpm_distance(d):
    """Flywheel RPM curve fitting (Flywheel.java:190)"""
    val = 957.2952559300876 + 14.312109862671662 * d
    return max(0, min(Constants.MAX_RPM, val))

class KinematicsSimulator:
    """
    Simulates the KinematicsSolver.java algorithm for stationary robot
    """
    def __init__(self):
        # Current state
        self.theta_launch = (Constants.LAUNCH_ANGLE_MIN + Constants.LAUNCH_ANGLE_MAX) / 2
        self.v_launch = 0
        self.alpha_launch = 0

        # Cached vectors (simplified for stationary case)
        self.s0 = [Constants.WHEEL_X, Constants.WHEEL_Y]
        self.s_goal = [0, Constants.GOAL_HEIGHT]

    def set_goal_position(self, is_red_alliance=True):
        """Calculate goal position (KinematicsSolver.java:89-100)"""
        i = 1 if is_red_alliance else -1
        G_x = Constants.HALF_F + (Constants.HALF_F - Constants.O_GOAL_X) * i
        G_y = Constants.HALF_F + Constants.HALF_F + Constants.O_GOAL_Y
        return G_x, G_y

    def compute_forward_kinematics(self, theta, v, distance_to_goal):
        """
        Simplified forward kinematics for stationary robot
        Returns: (x_at_goal, y_at_goal, valid)
        """
        # Launch point
        s0_x = self.s0[0]
        s0_y = self.s0[1]

        # Goal point (relative to launch)
        s_goal_x = distance_to_goal
        s_goal_y = Constants.GOAL_HEIGHT

        # Velocity components
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        v0_x = v * cos_theta
        v0_y = v * sin_theta

        # Time to reach goal x
        if abs(v0_x) < 0.001:
            return 0, 0, False

        t = (s_goal_x - s0_x) / v0_x

        # Y position at goal
        y_at_goal = s0_y + v0_y * t + 0.5 * Constants.GRAVITY * t * t

        return s_goal_x, y_at_goal, True

    def theta_3pt(self, target_x, target_y, goal_x, goal_y):
        """
        Calculate launch angle to hit both target point and goal (KinematicsSolver.java:239-250)
        θ_3pt finds angle for ball to pass through (tx, ty) and reach (dx, dy)

        Formula: atan((dx²*ty - tx²*dy) / (dx*tx*(dx - tx)))
        """
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
        """
        Calculate velocity from angle to reach goal (KinematicsSolver.java:226-229)
        v = dx / (cos(θ) * sqrt((2/a_G) * (s_goal.y - s0.y - dx*tan(θ))))
        """
        s0_x = self.s0[0]
        s0_y = self.s0[1]

        dx = goal_x - s0_x

        # discriminant = s_goal.y - s0.y - dx*tan(θ)
        discriminant = goal_y - s0_y - dx * tan_theta

        # a_G is negative, so (2/a_G) * discriminant must be positive
        if discriminant >= 0:
            return float('inf')  # Cannot reach with this angle

        val = (2.0 / Constants.GRAVITY) * discriminant
        if val <= 0:
            return float('inf')

        return dx / (cos_theta * math.sqrt(val))

    def calculate_target(self, distance_to_goal, rim_x, rim_y):
        """
        Implements calculateTarget_v_θ_α() algorithm (KinematicsSolver.java:259-314)
        For stationary robot, simplified version
        """
        # Goal position
        goal_x = distance_to_goal
        goal_y = Constants.GOAL_HEIGHT

        # Initialize
        theta_launch = (Constants.LAUNCH_ANGLE_MIN + Constants.LAUNCH_ANGLE_MAX) / 2
        v_launch = 100  # Initial guess
        alpha_launch = 0

        # Iterate (simplified from Java version)
        for iteration in range(5):
            # Calculate angle to clear rim (3-point shot)
            # Rim clearance point
            rim_clearance_y = rim_y + Constants.BALL_RADIUS + Constants.RIM_CLEARANCE

            theta_to_rim = self.theta_3pt(rim_x, rim_clearance_y, goal_x, goal_y)

            # Ensure angle is within bounds
            if theta_to_rim < Constants.LAUNCH_ANGLE_MIN:
                theta_to_rim = Constants.LAUNCH_ANGLE_MIN
            elif theta_to_rim > Constants.LAUNCH_ANGLE_MAX:
                theta_to_rim = Constants.LAUNCH_ANGLE_MAX

            # Calculate velocity for this angle
            cos_theta = math.cos(theta_to_rim)
            sin_theta = math.sin(theta_to_rim)

            if cos_theta <= 0.01:
                continue

            tan_theta = sin_theta / cos_theta
            v_needed = self.v1(cos_theta, tan_theta, goal_x, goal_y)

            if v_needed <= 0 or v_needed > Constants.MAX_VELOCITY:
                # Try to find a valid velocity by adjusting angle
                # Use midpoint between current and max
                theta_to_rim = (theta_to_rim + Constants.LAUNCH_ANGLE_MAX) / 2
                cos_theta = math.cos(theta_to_rim)
                sin_theta = math.sin(theta_to_rim)
                if cos_theta > 0.01:
                    tan_theta = sin_theta / cos_theta
                    v_needed = self.v1(cos_theta, tan_theta, goal_x, goal_y)

            if 0 < v_needed <= Constants.MAX_VELOCITY:
                theta_launch = theta_to_rim
                v_launch = v_needed

        # Check if solution is valid
        # Verify we can actually reach the goal
        _, y_at_goal, valid = self.compute_forward_kinematics(theta_launch, v_launch, distance_to_goal)

        if not valid or abs(y_at_goal - Constants.GOAL_HEIGHT) > 1.0:
            # Try alternative: use maximum launch angle
            theta_launch = Constants.LAUNCH_ANGLE_MAX
            cos_theta = math.cos(theta_launch)
            sin_theta = math.sin(theta_launch)
            tan_theta = sin_theta / cos_theta
            v_launch = self.v1(cos_theta, tan_theta, goal_x, goal_y)

            if v_launch <= 0 or v_launch > Constants.MAX_VELOCITY:
                return None, None, False

        return theta_launch, v_launch, True

    def solve(self, robot_x, robot_y, robot_heading):
        """
        Calculate shooting parameters for stationary robot
        Returns: (turret_angle_deg, hood_servo_angle, flywheel_rpm, valid)
        """
        # Goal position (red alliance)
        G_x, G_y = self.set_goal_position(True)

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

        # Rim position (simplified)
        # Rim is at approximately 70.75, 70.75 for red alliance
        rim_dist_from_goal = 15  # Approximate
        rim_x = distance_to_goal - rim_dist_from_goal
        rim_y = Constants.RIM_HEIGHT

        # Calculate kinematics
        theta_launch, v_launch, valid = self.calculate_target(distance_to_goal, rim_x, rim_y)

        if not valid or theta_launch is None:
            return turret_angle_deg, Constants.HOOD_MIN, 0, False

        # Convert to outputs
        hood_servo = launch_rad_to_servo(theta_launch)
        rpm = v_launch * Constants.RPM_PER_SEC_IN

        return turret_angle_deg, hood_servo, rpm, True

def main():
    print("=" * 130)
    print("CORRECTED TREND ANALYSIS: KinematicsSolver vs Curve Fitting")
    print("Robot is STATIONARY (vx=vy=angVel=0) on a 144x144 field")
    print(f"Goal position: ({Constants.GOAL_X}, {Constants.GOAL_Y}) - Center of far wall")
    print("=" * 130)
    print()
    print(f"{'Point':<10} | {'Dist':<6} | {'Turret (deg)':<26} | {'Hood Servo':<28} | {'Flywheel RPM':<26} |")
    print(f"{'(x,y)':<10} | {'(in)':<6} | {'Kinem.':<12} {'Curve':<12} | {'Kinem.':<13} {'Curve':<12} | {'Kinem.':<12} {'Curve':<11} |")
    print("-" * 130)

    # Test points
    test_points = [
        (30, 40), (72, 40), (114, 40),
        (30, 60), (72, 60), (114, 60),
        (30, 80), (72, 80), (114, 80),
        (40, 100), (72, 100), (104, 100),
        (50, 110), (72, 110), (94, 110),
        (50, 120), (72, 120), (94, 120),
        (60, 125), (72, 125), (84, 125),
        (20, 80), (124, 80),
    ]

    solver = KinematicsSimulator()
    results = []

    for x, y in test_points:
        # Heading to face goal
        dx = Constants.GOAL_X - x
        dy = Constants.GOAL_Y - y
        heading = math.atan2(dy, dx)
        distance = math.sqrt(dx*dx + dy*dy)

        # Kinematics
        kin_turret, kin_hood, kin_rpm, valid = solver.solve(x, y, heading)

        # Curve fitting
        curve_hood = curve_hood_distance(distance)
        curve_rpm = curve_rpm_distance(distance)
        curve_turret = 0.0

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
    print("\n* = Kinematics: No valid solution found within constraints")

    # Calculate statistics
    valid_results = [r for r in results if r['valid']]
    if valid_results:
        avg_hood_kin = sum(r['kin_hood'] for r in valid_results) / len(valid_results)
        avg_rpm_kin = sum(r['kin_rpm'] for r in valid_results) / len(valid_results)
        avg_hood_curve = sum(r['curve_hood'] for r in results) / len(results)
        avg_rpm_curve = sum(r['curve_rpm'] for r in results) / len(results)

        print(f"\nAVERAGES (valid kinematics solutions only):")
        print(f"  Hood: Kinematics={avg_hood_kin:.1f}°, Curve={avg_hood_curve:.1f}°")
        print(f"  RPM:  Kinematics={avg_rpm_kin:.1f}, Curve={avg_rpm_curve:.1f}")

    print("\n" + "=" * 130)
    print("LAUNCH ANGLE MAPPING:")
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    print(f"  θ_launchMax = {max_deg:.1f}° → Hood.MIN = {Constants.HOOD_MIN}°")
    print(f"  θ_launchMin = {min_deg:.1f}° → Hood.MAX = {Constants.HOOD_MAX}°")
    print(f"  Scale: {112/(max_deg-min_deg):.2f} servo degrees per launch degree (INVERSE relationship)")
    print()
    print("NOTE: Hood servo angle is INVERSE to launch angle:")
    print("  - Higher launch angle (steeper) → Lower servo value (hood more vertical)")
    print("  - Lower launch angle (flatter) → Higher servo value (hood more horizontal)")
    print("=" * 130)

if __name__ == "__main__":
    main()
