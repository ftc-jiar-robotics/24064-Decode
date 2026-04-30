package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OctoQuadConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;

public class Constants {



    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.3)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125,0.000025,0.01125,0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9,0,0.015,0.003))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0076, 0, 0.00004, 0.6, 0.05))
            .forwardZeroPowerAcceleration(-27.437689317630763)
            .lateralZeroPowerAcceleration(-67.38974604416983)
            .centripetalScaling(0.00015);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("left front")
            .leftRearMotorName("left back")
            .rightFrontMotorName("right front")
            .rightRearMotorName("right back")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80.12127637487697)
            .yVelocity(59.382353054256896)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(Common.FORWARD_POD_Y)
            .strafePodX(Common.STRAFE_POD_X)
            .yawScalar(Common.IMU_YAW_SCALAR)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static OctoQuadConstants octoQuadConstants = new OctoQuadConstants()
            .imuScalar(1.01025f);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            150,
            0.95,
            10,
            .95
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
