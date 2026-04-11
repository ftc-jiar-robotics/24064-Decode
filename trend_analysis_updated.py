#!/usr/bin/env python3
"""
UPDATED Trend Analysis: KinematicsSolver vs Curve Fitting
Robot is stationary (vx=vy=angVel=0) on a 144x144 field

UPDATED CONSTANTS from latest code:
- Hood.java: MIN=70, MAX=160, PHYSICAL_MAX=195
- KinematicsSolver.java: θ_launchMax=68°, scale=(13/127)*(60/20)
- Flywheel.java: RPM_PER_SEC_IN=8.17067
"""

import math

class Constants:
    """UPDATED physical and mechanical constants from latest Java code"""
    # Field
    GOAL_X = 72
    GOAL_Y = 144

    # Hood servo limits (Hood.java:25-27) - UPDATED
    HOOD_MIN = 70  # was 68
    HOOD_MAX = 160  # was 180
    HOOD_PHYSICAL_MAX = 195  # was 200

    # Launch angle limits (KinematicsSolver.java:39-40) - UPDATED
    # θ_launchMax = toRadians(68) - changed from 48.5!
    LAUNCH_ANGLE_MAX = math.radians(68)  # was 48.5
    # Scale factor changed from (15/131)*(40/20) to (13/127)*(60/20)
    SERVO_PER_LAUNCH_DEG = (13.0/127) * (60.0/20)  # ~0.3071
    LAUNCH_ANGLE_MIN = LAUNCH_ANGLE_MAX - math.radians((HOOD_MAX - HOOD_MIN) * SERVO_PER_LAUNCH_DEG)
    # LAUNCH_ANGLE_MIN = 68° - 27.7° = 40.3°

    # Flywheel limits (Flywheel.java:39,52) - UPDATED
    MAX_RPM = 4000
    RPM_PER_SEC_IN = 8.17067  # was 8.21238
    MAX_VELOCITY = MAX_RPM / RPM_PER_SEC_IN  # ~489.7 in/s

    # Turret offset (Common.java:69)
    TURRET_OFFSET_Y = -2.559

    # Kinematics (KinematicsSolver.java:44-58)
    GRAVITY = -386.0886
    GOAL_HEIGHT = 40
    RIM_HEIGHT = 38.75
    RIM_CLEARANCE = 0.75
    BALL_RADIUS = 2.5
    WHEEL_RADIUS_PHYSICAL = 1.5
    COMPRESSION = 6/25.4
    WHEEL_RADIUS = WHEEL_RADIUS_PHYSICAL - COMPRESSION
    C = WHEEL_RADIUS + BALL_RADIUS

    # Wheel position (KinematicsSolver.java:57)
    WHEEL_X = 3.64584291339
    WHEEL_Y = 10.611220472440944

    # Goal calculation (KinematicsSolver.java:33,53,88-99)
    HALF_F = 141.5 / 2
    O_GOAL_X = 5
    O_GOAL_Y = -2.5

def launch_rad_to_servo(rad):
    """
    Convert launch radians to hood servo angle (Hood.java:61-65)

    Hood.launchRadiansToServoAngle:
        scaleFactor = (MAX - MIN) / (toDegrees(θ_launchMax) - toDegrees(θ_launchMin))
        return MIN + toDegrees(θ_launchMax - targetRadians) * scaleFactor

    With NEW values:
    - θ_launchMax = 68° → Hood.MIN = 70° (steepest)
    - θ_launchMin = 40.3° → Hood.MAX = 160° (shallowest)
    """
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    return Constants.HOOD_MIN + (math.degrees(Constants.LAUNCH_ANGLE_MAX - rad)) * scale_factor

def servo_to_launch_rad(servo_angle):
    """Inverse: servo angle to launch radians"""
    max_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    scale_factor = (Constants.HOOD_MAX - Constants.HOOD_MIN) / (max_deg - min_deg)
    launch_deg = max_deg - (servo_angle - Constants.HOOD_MIN) / scale_factor
    return math.radians(launch_deg)

def curve_hood_distance(d):
    """Hood curve fitting (Hood.java:50)"""
    # 21.255 + 1.552*d - 0.00445*d^2
    val = 21.25528184671319 + 1.551657839403464 * d - 0.00445309317429351 * d * d
    return max(Constants.HOOD_MIN, min(Constants.HOOD_MAX, val))

def curve_rpm_distance(d):
    """Flywheel RPM curve fitting (Flywheel.java:189)"""
    # 957.295 + 14.312*d
    val = 957.2952559300876 + 14.312109862671662 * d
    return max(0, min(Constants.MAX_RPM, val))

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
        v_launch = 200  # Initial guess

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
                # Adjust angle to find valid velocity
                # Try shallower angle (higher servo value = closer to MAX)
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

        hood_servo = launch_rad_to_servo(theta_launch)
        rpm = v_launch * Constants.RPM_PER_SEC_IN

        return hood_servo, rpm, True

    def solve(self, robot_x, robot_y, robot_heading):
        """Calculate shooting parameters"""
        # Goal position
        i = 1  # Red alliance
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

def main():
    print("=" * 130)
    print("UPDATED TREND ANALYSIS: KinematicsSolver vs Curve Fitting")
    print("Robot is STATIONARY (vx=vy=angVel=0) on a 144x144 field")
    print("=" * 130)
    print()

    # Show new constants
    max_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MAX)
    min_launch_deg = math.degrees(Constants.LAUNCH_ANGLE_MIN)
    print("UPDATED CONSTANTS:")
    print(f"  Hood.java: MIN={Constants.HOOD_MIN}, MAX={Constants.HOOD_MAX}")
    print(f"  KinematicsSolver.java:")
    print(f"    θ_launchMax = {max_launch_deg} deg → Hood.MIN = {Constants.HOOD_MIN}")
    print(f"    θ_launchMin = {min_launch_deg:.1f} deg → Hood.MAX = {Constants.HOOD_MAX}")
    print(f"  Flywheel.java: RPM_PER_SEC_IN = {Constants.RPM_PER_SEC_IN}")
    print()

    print(f"{'Point':<10} | {'Dist':<6} | {'Turret (deg)':<26} | {'Hood Servo':<28} | {'Flywheel RPM':<26} |")
    print(f"{'(x,y)':<10} | {'(in)':<6} | {'Kinem.':<12} {'Curve':<12} | {'Kinem.':<13} {'Curve':<12} | {'Kinem.':<12} {'Curve':<11} |")
    print("-" * 130)

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
        dx = Constants.GOAL_X - x
        dy = Constants.GOAL_Y - y
        heading = math.atan2(dy, dx)

        kin_turret, kin_hood, kin_rpm, valid, distance = solver.solve(x, y, heading)

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

    # Statistics
    valid_results = [r for r in results if r['valid']]
    if valid_results:
        avg_hood_kin = sum(r['kin_hood'] for r in valid_results) / len(valid_results)
        avg_rpm_kin = sum(r['kin_rpm'] for r in valid_results) / len(valid_results)
        avg_hood_curve = sum(r['curve_hood'] for r in results) / len(results)
        avg_rpm_curve = sum(r['curve_rpm'] for r in results) / len(results)

        print(f"\nAVERAGES (valid kinematics only):")
        print(f"  Hood: Kinematics={avg_hood_kin:.1f} deg, Curve={avg_hood_curve:.1f} deg")
        print(f"  RPM:  Kinematics={avg_rpm_kin:.1f}, Curve={avg_rpm_curve:.1f}")

    print("\n" + "=" * 130)
    print("LAUNCH ANGLE TO HOOD SERVO MAPPING (UPDATED):")
    print(f"  θ_launchMax = {max_launch_deg} deg  → Hood.MIN = {Constants.HOOD_MIN} deg (steepest)")
    print(f"  θ_launchMin = {min_launch_deg:.1f} deg → Hood.MAX = {Constants.HOOD_MAX} deg (shallowest)")
    print(f"  Scale factor: {(Constants.HOOD_MAX-Constants.HOOD_MIN)/(max_launch_deg-min_launch_deg):.2f} servo deg per launch deg")
    print()
    print("INVERSE RELATIONSHIP (still applies):")
    print("  Higher launch angle (steeper shot) → Lower servo value (MIN=70)")
    print("  Lower launch angle (flatter shot) → Higher servo value (MAX=160)")
    print("=" * 130)

if __name__ == "__main__":
    main()
