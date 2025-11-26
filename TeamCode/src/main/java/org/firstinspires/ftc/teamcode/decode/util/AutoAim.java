package org.firstinspires.ftc.teamcode.decode.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AutoAim {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor processor;

    private int activeTargetId;
    private final double FX = 577.345, FY = 577.345, CX = 320.0, CY = 180.0;

    // distance where we're near/far from the tag
    private static double NEAR_DIST_IN = 24.0;
    private static double FAR_DIST_IN  = 96.0;
    // allowed decimation range
    private static float  MIN_DECIM    = 1.0f;
    private static float  MAX_DECIM    = 3.0f;
    //decimation we want when near/far
    private static float  NEAR_DECIM   = 3.0f;
    private static float  FAR_DECIM    = 1.2f;



    private AprilTagDetection cached = null;

    /**
     * @param hw              FTC HardwareMap
     * @param webcamName      the configured camera name (e.g., "ArduCam")
     */
    public AutoAim(HardwareMap hw, String webcamName) {
        setAlliance();

        AprilTagProcessor.Builder procB = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(false)
                .setLensIntrinsics(FX, FY, CX, CY)
                .setCameraPose(new Position(DistanceUnit.INCH, Common.CAM_OFFSET_X, Common.CAM_OFFSET_Y, Common.CAM_HEIGHT, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, Common.CAM_PITCH - 90, 0, 0 ))
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawCubeProjection(false);

        

        processor = procB.build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, webcamName))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(processor)
                .build();
    }

    /** Change alliance (and active target ID) on the fly if needed. */
    public void setAlliance() {
        this.activeTargetId = (Common.isRed) ? Common.RED_GOAL_ID : Common.BLUE_GOAL_ID;
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
    public void updateDecimation() {
        double r = (cached == null) ? Double.POSITIVE_INFINITY : cached.ftcPose.range;
        double t = (r - NEAR_DIST_IN) / (FAR_DIST_IN - NEAR_DIST_IN);
        t = Math.max(0.0, Math.min(1.0, t));
        float dec = (float)(NEAR_DECIM + t * (FAR_DECIM - NEAR_DECIM));
        dec = Math.max(MIN_DECIM, Math.min(MAX_DECIM, dec));
        processor.setDecimation(dec);
    }




    public Pose getTurretPosePedro() {
        double x = cached.robotPose.getPosition().y + 72;
        double y = 72 - cached.robotPose.getPosition().x;
        double headingDeg = cached.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        return new Pose(x, y, headingDeg);
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
