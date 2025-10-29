package org.firstinspires.ftc.teamcode.decode.util;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AutoAim {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor processor;
    private final double tagSizeMeters;

    private final int redTargetId;
    private final int blueTargetId;
    private int activeTargetId;
    private final double FX = 577.345, FY = 577.345, CX = 320.0, CY = 180.0;


    private AprilTagDetection cached = null;

    /**
     * @param hw              FTC HardwareMap
     * @param isRed           if true → sets AprilTag ID to red alliance at init
     * @param redTargetId     The tag ID to track when on RED alliance
     * @param blueTargetId    The tag ID to track when on BLUE alliance
     * @param tagSizeMeters   Physical tag size (edge length) in meters (e.g., 0.166 for 166 mm)
     * @param webcamName      the configured camera name (e.g., "ArduCam")
     */
    public AutoAim(HardwareMap hw, boolean isRed, int redTargetId, int blueTargetId, double tagSizeMeters, String webcamName) {
        this.redTargetId = redTargetId;
        this.blueTargetId = blueTargetId;
        this.activeTargetId = (isRed) ? redTargetId : blueTargetId;
        this.tagSizeMeters = tagSizeMeters;

        AprilTagProcessor.Builder procB = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawCubeProjection(false);

        procB.setLensIntrinsics(FX, FY, CX, CY);

        processor = procB.build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(processor)
                .build();
    }

    /** Compute robot (X,Y) on the field in inches; angles in radians.
     *  Requires: call isTargetDetected() first so 'cached' is set.
     *  turretYawRad = absolute turret/camera yaw in field frame (radians)
     */
    public Pose getRobotPos(double turretYawRad) {
        if (cached == null) return new Pose(Double.NaN, Double.NaN, turretYawRad);

        // Tag constants
        double[] tag = (cached.id == 20) ? Common.TAG_POSES[0] : (cached.id == 24) ? Common.TAG_POSES[1] : null; // {id, tagX, tagY, tagHeight}
        if (tag == null) return new Pose(Double.NaN, Double.NaN, turretYawRad);
        final double tagX = tag[1];
        final double tagY = tag[2];
        final double tagHeight = tag[3];

        //  Vision inputs
        final double camDistIn = Math.max(cached.ftcPose.range, 0.0);         // inches, slanted lens→tag
        final double camBearingRad = Math.toRadians(cached.ftcPose.yaw);      // degrees -> radians (CCW+)

        //  1) Flatten slanted distance to ground distance
        double dz = Common.CAM_HEIGHT - tagHeight;                         // inches
        double dFlatSq = camDistIn*camDistIn - dz*dz;
        double dFlat = dFlatSq > 0 ? Math.sqrt(dFlatSq) : 0.0;

        //  2) Tag in camera/turret frame (+Y fwd, +X right)
        double tagRelX = -dFlat * Math.sin(camBearingRad); // left = +bearing -> negative X
        double tagRelY =  dFlat * Math.cos(camBearingRad);

        //  3) Camera field position (rotate local -> field by turretYaw)
        double c = Math.cos(turretYawRad), s = Math.sin(turretYawRad);
        double camX = tagX - (tagRelX * c - tagRelY * s);
        double camY = tagY - (tagRelX * s + tagRelY * c);

        //  4) Turret pivot (remove rotated camera offset)
        double turX = camX - (Common.CAM_OFFSET_X * c - Common.CAM_OFFSET_Y * s);
        double turY = camY - (Common.CAM_OFFSET_X * s + Common.CAM_OFFSET_Y * c);

        //  5) Robot center (apply turret -> robot Y offset;  negative bc turret is behind center)
        double robotX = turX + Common.TURRET_OFFSET_Y * Math.sin(turretYawRad);
        double robotY = turY - Common.TURRET_OFFSET_Y * Math.cos(turretYawRad);

        return new Pose(robotX, robotY, turretYawRad);
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
        final double goalOffX_cam = Common.GOAL_FROM_TAG_X * c - Common.GOAL_FROM_TAG_Y * s;
        final double goalOffY_cam = Common.GOAL_FROM_TAG_X * s + Common.GOAL_FROM_TAG_Y * c;

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
    public void setAlliance(boolean isRed) {
        this.activeTargetId = (isRed) ? redTargetId : blueTargetId;
    }

    /** Returns true if the selected target tag (by alliance) is currently detected. */
    public boolean isTargetDetected() {
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
