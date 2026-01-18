package org.firstinspires.ftc.teamcode.decode.util;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.BLUE_GOAL_ID;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.COLOR_SENSOR_UPDATE_LOOPS;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.RED_GOAL_ID;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

@Config
@Configurable
public class ArduCam {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor processor;

    public static int DESIRED_THREADS = 5;

    private long readTime = Long.MIN_VALUE;

    private int activeTargetId;
    private final double FX = 540.7083, FY = 543.8710, CX = 312.9142, CY = 269.4617;

    private final Queue<Pose> visionSamplePoses = new LinkedList<>();
    private double[] visionVariances = new double[2];

    //decimation we want when near/far
    private static int    X_BUFFER_SIZE = 3;
    private static int    Y_BUFFER_SIZE = 3;

    private Pose latestPose;
    public static int MAX_VARIANCE_SIZE = 4;

    private MovingAverageFilter xFilter = new MovingAverageFilter(new MovingAverageGains(X_BUFFER_SIZE));
    private MovingAverageFilter yFilter = new MovingAverageFilter(new MovingAverageGains(Y_BUFFER_SIZE));


    private AprilTagDetection cached = null;

    /**
     * @param hw              FTC HardwareMap
     * @param webcamName      the configured camera name (e.g., "ArduCam")
     */
    public ArduCam(HardwareMap hw, String webcamName) {
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

//        visionPortal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Auto);
    }

    public static double[] getVariance(Queue<Pose> queue) {
        double meanY = 0;
        double meanX = 0;
        double xVariance = 0;
        double yVariance = 0;

        double size = queue.size();

        for (Pose p : queue) {
            meanY += p.getY();
            meanX += p.getX();
        }

        meanY /= size;
        meanX /= size;

        for (Pose p : queue) {
            double pX = p.getX();
            double pY = p.getY();

            xVariance += (pX - meanX) * (pX - meanX);
            yVariance += (pY - meanY) * (pY - meanY);
        }

        xVariance /= size - 1;
        yVariance /= size - 1;

        return new double[]{xVariance, yVariance};
    }

    /** Change alliance (and active target ID) on the fly if needed. */
    public void setAlliance() {
        this.activeTargetId = (Common.isRed) ? RED_GOAL_ID : Common.BLUE_GOAL_ID;
    }

    /** Returns true if the selected target tag (by alliance) is currently detected. */
    public boolean detectTarget() {
        cached = null;
        List<AprilTagDetection> dets = processor.getDetections();
        if (dets == null || dets.isEmpty()) return false;

        // since the field has only one of this ID, first match is enough
        for (AprilTagDetection d : dets) {
            if (d.id == RED_GOAL_ID || d.id == BLUE_GOAL_ID) {

            if (visionSamplePoses.size() > MAX_VARIANCE_SIZE) visionSamplePoses.remove();

            cached = d;
            readTime = System.nanoTime();
            if (cached.robotPose != null && cached.robotPose.getPosition() != null) visionSamplePoses.add(new Pose(cached.robotPose.getPosition().x, cached.robotPose.getPosition().y));

            visionVariances = getVariance(visionSamplePoses);

            return true;
            }
        }
        return false;
    }

    public long getStaleness() {

        return System.nanoTime() - readTime;
    }

    public double[] getVariances() {
        return visionVariances;
    }

    public Pose getTurretPosePedro() {
        if (getStaleness() > Robot.MAX_STALENESS) {
            xFilter.reset();
            yFilter.reset();
        }

        if (cached != null && cached.robotPose != null && cached.robotPose.getPosition() != null) {
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
