package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.29445)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125,0.000025,0.01125,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.67,0,0.015,0.035))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0067, 0, 0.00002, 0.6, 0.07))
            .forwardZeroPowerAcceleration(-35.3036594008226)
            .lateralZeroPowerAcceleration(-77.0797908371733)
            .centripetalScaling(0.00035);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("left front")
            .leftRearMotorName("left back")
            .rightFrontMotorName("right front")
            .rightRearMotorName("right back")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.75557101993111)
            .yVelocity(64.10822686623399)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(Common.FORWARD_POD_Y)
            .strafePodX(Common.STRAFE_POD_X)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            150,
            0.95,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
