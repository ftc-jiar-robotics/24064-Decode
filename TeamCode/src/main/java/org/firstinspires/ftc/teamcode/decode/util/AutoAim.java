package org.firstinspires.ftc.teamcode.decode.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AutoAim {
    private VisionPortal visionPortal;
    private CameraName camera;
    private AprilTagProcessor processor;
    private final int redTargetId;
    private final int blueTargetId;
    private int activeTargetId;
    private double turretYawRad;
    public static YawPitchRollAngles cameraOrientation;
    public static Position cameraRelativePos;
    private final double FX = 577.345, FY = 577.345, CX = 320.0, CY = 180.0;


    private AprilTagDetection cached = null;

    /**
     * @param hw              FTC HardwareMap
     * @param webcamName      the configured camera name (e.g., "ArduCam")
     */
    public AutoAim(HardwareMap hw, String webcamName) {
        this.redTargetId = Common.RED_GOAL_ID;
        this.blueTargetId = Common.BLUE_GOAL_ID;
        setAlliance();
//        AprilTagProcessor.Builder procB = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(false)
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setDrawCubeProjection(false)
//                .setLensIntrinsics(FX, FY, CX, CY);
//
//        processor = procB.build();

        processor = new AprilTagProcessorImpl(this::robotInCameraFrame, FX, FY, CX, CY, DistanceUnit.INCH, AngleUnit.RADIANS,
                AprilTagGameDatabase.getCurrentGameTagLibrary(), false, false, false, false,
                AprilTagProcessor.TagFamily.TAG_36h11, 3, false);



        camera = hw.get(WebcamName.class, webcamName);
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(processor)
                .build();
    }

    public void setTurretYawRad(double turretYawRad) {
        this.turretYawRad = turretYawRad;
    }

    /** Compute robot (X,Y) on the field in inches; angles in radians.
     *  Requires: call isTargetDetected() first so 'cached' is set.
     *  turretYawRad = absolute turret/camera yaw in field frame (radians)
     */
    public Pose getRobotPos() {
        return new Pose(cached.robotPose.getPosition().x, cached.robotPose.getPosition().y, cached.robotPose.getOrientation().getYaw());
    }

    public OpenGLMatrix robotInCameraFrame() {

        cameraRelativePos = new Position(DistanceUnit.INCH, 0 ,0 ,Common.CAM_HEIGHT, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, -1.0 * Turret.standardHeadingRadToTurretHeading(Math.toDegrees(turretYawRad)), (-90 + 10),0,0);
        double camDist = Math.sqrt(Common.CAM_OFFSET_Y * Common.CAM_OFFSET_Y  + Common.CAM_OFFSET_X * Common.CAM_OFFSET_X);
        double camRot = turretYawRad + Math.atan2(Common.CAM_OFFSET_X,Common.CAM_OFFSET_Y);
        cameraRelativePos.x = Math.cos(camRot) * camDist;
        cameraRelativePos.y = Math.sin(camRot) * camDist - Common.TURRET_OFFSET_Y;
        OpenGLMatrix cameraRotationMatrix = new Orientation(
                AxesReference.INTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES,
                (float) cameraOrientation.getYaw(AngleUnit.DEGREES),
                (float) cameraOrientation.getPitch(AngleUnit.DEGREES),
                (float) cameraOrientation.getRoll(AngleUnit.DEGREES),
                cameraOrientation.getAcquisitionTime())
                .getRotationMatrix();

        return OpenGLMatrix.identityMatrix()
                .translated(
                        (float) cameraRelativePos.toUnit(DistanceUnit.INCH).x,
                        (float) cameraRelativePos.toUnit(DistanceUnit.INCH).y,
                        (float) cameraRelativePos.toUnit(DistanceUnit.INCH).z)
                .multiplied(cameraRotationMatrix)
                .inverted();
    }



    /** Turret-frame yaw correction so the FLYWHEEL points at the GOAL (not the tag).
     *  Requires: call isTargetDetected() first so 'cached' is set.
     *  Requires: turretYawRad (radians) to rotate field offset into camera frame.
     *  Returns radians in (-π, π].
     */
    public double getYawOffsetRadians(double turretYawRad) {
        if (cached == null) return 0.0;

        double[] tag = (cached.id == 20) ? Common.TAG_POSES[0]
                : (cached.id == 24) ? Common.TAG_POSES[1]
                : null;
        if (tag == null) return 0.0;

        final double tagHeight = tag[3]; // inches

        // Vision inputs (inches & degrees)
        final double camDistIn = Math.max(cached.ftcPose.range, 0.0);  // slanted lens -> tag distance
        final double camBearingRad = Math.toRadians(cached.ftcPose.yaw);   // CCW+, camera-forward frame

        // --- 1) Flatten slanted range to ground distance (use height difference)
        final double dz =    Common.CAM_HEIGHT - tagHeight; // inches
        final double dFlatSq = camDistIn*camDistIn - dz*dz;
        final double dFlat =   (dFlatSq > 0) ? Math.sqrt(dFlatSq) : 0.0;

        // --- 2) Tag position in turret/camera frame (+Y fwd, +X right)
        final double tagX_cam = -dFlat * Math.sin(camBearingRad);  // left bearing → negative X
        final double tagY_cam =   dFlat * Math.cos(camBearingRad);

        // --- 3) Rotate field offset (goal-from-tag) into the turret/camera frame
        // local = R(-turretYaw) * fieldVector
        final double c = Math.cos(-turretYawRad), s = Math.sin(-turretYawRad);
        final double goalOffX_cam = (Common.GOAL_POSES[Common.isRed ? 1 : 0][1] - Common.TAG_POSES[Common.isRed ? 1 : 0][1]) * c - (Common.GOAL_POSES[Common.isRed ? 1 : 0][2] - Common.TAG_POSES[Common.isRed ? 1 : 0][2]) * s;
        final double goalOffY_cam = (Common.GOAL_POSES[Common.isRed ? 1 : 0][1] - Common.TAG_POSES[Common.isRed ? 1 : 0][1]) * s + (Common.GOAL_POSES[Common.isRed ? 1 : 0][2] - Common.TAG_POSES[Common.isRed ? 1 : 0][2]) * c;

        // --- 4) Goal point in camera frame = tag vector + rotated offset
        final double goalX_cam = tagX_cam + goalOffX_cam;
        final double goalY_cam = tagY_cam + goalOffY_cam;

        // --- 5) Vector from FLYWHEEL to GOAL (flywheel offset is defined in the same local frame)
        final double relX = goalX_cam - Common.FLYWHEEL_OFFSET_X;
        final double relY = goalY_cam - Common.FLYWHEEL_OFFSET_Y;

        // --- 6) Yaw correction = angle(flywheel - > goal) - angle(camera→forward)
        final double thetaCam = camBearingRad;           // camera's current bearing line
        final double thetaFly = Math.atan2(relX, relY);  // (+Y fwd, +X right) → atan2(x, y)
        return wrapRad(thetaFly - thetaCam);
    }

    /** Wrap angle to (-π, π]. */
    private static double wrapRad(double a) {
        double x = (a + Math.PI) % (2.0 * Math.PI);
        if (x < 0) x += 2.0 * Math.PI;
        return x - Math.PI;
    }




    /** Change alliance (and active target ID) on the fly if needed. */
    public void setAlliance() {
        this.activeTargetId = (Common.isRed) ? redTargetId : blueTargetId;
    }

    /** Returns true if the selected target tag (by alliance) is currently detected. */
    public boolean detectTarget() {
        cached = null;
        List<AprilTagDetection> dets = processor.getDetections();
        if (dets == null || dets.isEmpty()) return false;

        // since the field has only one of this ID, first match is enough
        for (AprilTagDetection d : dets) {
            if (d.id == activeTargetId) {
                cached = d;
                return true;
            }
        }
        return false;
    }


    /** Horizontal yaw error (deg) from camera center to the selected target tag. */
    public double getTargetYawDegrees() {
        return (cached != null) ? cached.ftcPose.yaw : 0.0;
    }

    /** (Optional) range, bearing, ID accessors */
    public AprilTagDetection getTargetDetection() { return cached; }
    public int getActiveTargetId() { return activeTargetId; }

    public void close() {
        if (visionPortal != null) visionPortal.close();
    }
}
