package org.firstinspires.ftc.teamcode.decode.util;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Configurable
public class AutoAim {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor processor;

    public static int DESIRED_THREADS = 5;

    private int activeTargetId;
    private final double FX = 540.7083, FY = 543.8710, CX = 312.9142, CY = 269.4617;

    //decimation we want when near/far
    private static int    X_BUFFER_SIZE = 3;
    private static int    Y_BUFFER_SIZE = 3;

    private Pose latestPose;

    private MovingAverageFilter xFilter = new MovingAverageFilter(new MovingAverageGains(X_BUFFER_SIZE));
    private MovingAverageFilter yFilter = new MovingAverageFilter(new MovingAverageGains(Y_BUFFER_SIZE));


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
                .setDrawAxes(true)
                .setLensIntrinsics(FX, FY, CX, CY)
                .setNumThreads(DESIRED_THREADS)
                .setCameraPose(new Position(DistanceUnit.INCH, Common.CAM_OFFSET_X, Common.CAM_OFFSET_Y, Common.CAM_HEIGHT, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, Common.CAM_PITCH - 90, 180, 0 ))
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawCubeProjection(true);

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

    public Pose getTurretPosePedro() {
        if (cached != null) {
            double x = cached.robotPose.getPosition().y + 72;
            double y = 72 - cached.robotPose.getPosition().x;
            double headingDeg = cached.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            return latestPose = new Pose(xFilter.calculate(x), yFilter.calculate(y), headingDeg);
        }
        return null;
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

    public void printTelemetry() {
        telemetry.addLine("ARDUCAM");
        if (latestPose != null) {
            telemetry.addData("arducam robot pose x (INCHES): ", latestPose.getX());
            telemetry.addData("arducam robot pose y (INCHES): ", latestPose.getY());
            telemetry.addData("arducam robot pose heading (DEGREES): ", latestPose.getHeading());
        }
    }

    }
