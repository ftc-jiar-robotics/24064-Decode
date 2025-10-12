package org.firstinspires.ftc.teamcode.decode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
                .setDrawCubeProjection(false);

        // Only set intrinsics if provided (>0). If you don't have them, yaw is still fine.

        procB.setLensIntrinsics(FX, FY, CX, CY);


        processor = procB.build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, webcamName))
                .addProcessor(processor)
                .build();
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
