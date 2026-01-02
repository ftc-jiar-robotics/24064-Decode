package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_LEFT_PIN0;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_RIGHT_PIN0;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_CAMERA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.util.ArduCam;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@TeleOp(name = "SensorLoopTime", group = "Debug")
@Config
public class SensorLoopTime extends LinearOpMode {

    // ====== DASHBOARD TOGGLES ======
    public static boolean USE_BULK     = true;
    public static boolean USE_COLOR    = true;
    public static boolean USE_PINS     = true;
    public static boolean USE_CAMERA   = true;
    public static boolean USE_PINPOINT = true;

    private BulkReader bulkReader;
    private DigitalChannel pin0Left, pin0Right;
    private ArduCam arducam;

    private static final String PINPOINT_NAME = "pinpoint";
    private GoBildaPinpointDriver pinpoint;

    // --- Stats containers ---
    private double minLoopMs, maxLoopMs, sumLoopMs;
    private double minGapMs,  maxGapMs,  sumGapMs;
    private int loopCount;

    private double minBulkMs, maxBulkMs, sumBulkMs;      private int bulkCount;
    private double minIntakeMs, maxIntakeMs, sumIntakeMs;private int intakeCount;
    private double minFeederMs, maxFeederMs, sumFeederMs;private int feederCount;
    private double minPinsMs, maxPinsMs, sumPinsMs;      private int pinsCount;
    private double minCamMs, maxCamMs, sumCamMs;         private int camCount;
    private double minPinpointMs, maxPinpointMs, sumPinpointMs; private int pinpointCount;

    // last values to detect Dashboard changes & reset stats
    private boolean lastUSE_BULK, lastUSE_COLOR, lastUSE_PINS, lastUSE_CAMERA, lastUSE_PINPOINT;

    private void resetStats() {
        minLoopMs = Double.POSITIVE_INFINITY; maxLoopMs = 0; sumLoopMs = 0;
        minGapMs  = Double.POSITIVE_INFINITY; maxGapMs  = 0; sumGapMs  = 0;
        loopCount = 0;

        minBulkMs = Double.POSITIVE_INFINITY; maxBulkMs = 0; sumBulkMs = 0; bulkCount = 0;
        minIntakeMs = Double.POSITIVE_INFINITY; maxIntakeMs = 0; sumIntakeMs = 0; intakeCount = 0;
        minFeederMs = Double.POSITIVE_INFINITY; maxFeederMs = 0; sumFeederMs = 0; feederCount = 0;
        minPinsMs   = Double.POSITIVE_INFINITY; maxPinsMs   = 0; sumPinsMs   = 0; pinsCount   = 0;
        minCamMs    = Double.POSITIVE_INFINITY; maxCamMs    = 0; sumCamMs    = 0; camCount    = 0;
        minPinpointMs = Double.POSITIVE_INFINITY; maxPinpointMs = 0; sumPinpointMs = 0; pinpointCount = 0;
    }

    private void detectToggleChangesAndReset() {
        if (USE_BULK     != lastUSE_BULK
                || USE_COLOR    != lastUSE_COLOR
                || USE_PINS     != lastUSE_PINS
                || USE_CAMERA   != lastUSE_CAMERA
                || USE_PINPOINT != lastUSE_PINPOINT) {
            resetStats();
        }
        lastUSE_BULK     = USE_BULK;
        lastUSE_COLOR    = USE_COLOR;
        lastUSE_PINS     = USE_PINS;
        lastUSE_CAMERA   = USE_CAMERA;
        lastUSE_PINPOINT = USE_PINPOINT;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Common.dashTelemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        bulkReader = new BulkReader(hardwareMap);

        pin0Left  = hardwareMap.digitalChannel.get(NAME_FEEDER_LEFT_PIN0);
        pin0Right = hardwareMap.digitalChannel.get(NAME_FEEDER_RIGHT_PIN0);
        pin0Left.setMode(DigitalChannel.Mode.INPUT);
        pin0Right.setMode(DigitalChannel.Mode.INPUT);

        arducam = new ArduCam(hardwareMap, NAME_TURRET_CAMERA);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);

        telemetry.addLine("Sensor Loop Time (Dashboard Config)");
        telemetry.addLine("Open Dashboard > Config tab to toggle:");
        telemetry.addLine("USE_BULK, USE_COLOR, USE_PINS, USE_CAMERA, USE_PINPOINT");
        telemetry.update();

        resetStats();
        lastUSE_BULK = USE_BULK;
        lastUSE_COLOR = USE_COLOR;
        lastUSE_PINS = USE_PINS;
        lastUSE_CAMERA = USE_CAMERA;
        lastUSE_PINPOINT = USE_PINPOINT;

        waitForStart();

        ElapsedTime opTimer = new ElapsedTime();
        opTimer.reset();

        long lastLoopStartNs = System.nanoTime();

        while (opModeIsActive()) {

            detectToggleChangesAndReset();

            LoopUtil.updateLoopCount();

            long loopStartNs = System.nanoTime();
            double gapMs = (loopStartNs - lastLoopStartNs) / 1e6;
            lastLoopStartNs = loopStartNs;

            double bulkMs = 0, intakeMs = 0, feederMs = 0, pinsMs = 0, camMs = 0, pinpointMs = 0;
            boolean leftState = false, rightState = false;
            boolean hasTarget = false;
            Pose turretPose = null;

            // Bulk
            if (USE_BULK) {
                long t0 = System.nanoTime();
                bulkReader.bulkRead();
                long t1 = System.nanoTime();
                bulkMs = (t1 - t0) / 1e6;
                minBulkMs = Math.min(minBulkMs, bulkMs);
                maxBulkMs = Math.max(maxBulkMs, bulkMs);
                sumBulkMs += bulkMs;
                bulkCount++;
            }

            // Color


            // Pins
            if (USE_PINS) {
                long t0 = System.nanoTime();
                leftState  = pin0Left.getState();
                rightState = pin0Right.getState();
                long t1 = System.nanoTime();
                pinsMs = (t1 - t0) / 1e6;

                minPinsMs = Math.min(minPinsMs, pinsMs);
                maxPinsMs = Math.max(maxPinsMs, pinsMs);
                sumPinsMs += pinsMs;
                pinsCount++;
            }

            // Camera
            if (USE_CAMERA) {
                long t0 = System.nanoTime();
                hasTarget = arducam.detectTarget();
                if (hasTarget) {
                    turretPose = arducam.getTurretPosePedro();
                }
                long t1 = System.nanoTime();
                camMs = (t1 - t0) / 1e6;

                minCamMs = Math.min(minCamMs, camMs);
                maxCamMs = Math.max(maxCamMs, camMs);
                sumCamMs += camMs;
                camCount++;
            }

            // Pinpoint
            if (USE_PINPOINT) {
                long t0 = System.nanoTime();
                pinpoint.update();
                long t1 = System.nanoTime();
                pinpointMs = (t1 - t0) / 1e6;

                minPinpointMs = Math.min(minPinpointMs, pinpointMs);
                maxPinpointMs = Math.max(maxPinpointMs, pinpointMs);
                sumPinpointMs += pinpointMs;
                pinpointCount++;
            }

            long loopEndNs = System.nanoTime();
            double loopMs = (loopEndNs - loopStartNs) / 1e6;

            loopCount++;
            minLoopMs = Math.min(minLoopMs, loopMs);
            maxLoopMs = Math.max(maxLoopMs, loopMs);
            sumLoopMs += loopMs;
            double avgLoopMs = sumLoopMs / loopCount;

            minGapMs = Math.min(minGapMs, gapMs);
            maxGapMs = Math.max(maxGapMs, gapMs);
            sumGapMs += gapMs;
            double avgGapMs = sumGapMs / loopCount;

            double avgBulkMs      = bulkCount      > 0 ? sumBulkMs      / bulkCount      : 0;
            double avgIntakeMs    = intakeCount    > 0 ? sumIntakeMs    / intakeCount    : 0;
            double avgFeederMs    = feederCount    > 0 ? sumFeederMs    / feederCount    : 0;
            double avgPinsMs      = pinsCount      > 0 ? sumPinsMs      / pinsCount      : 0;
            double avgCamMs       = camCount       > 0 ? sumCamMs       / camCount       : 0;
            double avgPinpointMs  = pinpointCount  > 0 ? sumPinpointMs  / pinpointCount  : 0;

            // === Telemetry ===
            telemetry.addLine("Dashboard toggles: USE_BULK, USE_COLOR, USE_PINS, USE_CAMERA, USE_PINPOINT");
            telemetry.addData("USE_BULK", USE_BULK);
            telemetry.addData("USE_COLOR", USE_COLOR);
            telemetry.addData("USE_PINS", USE_PINS);
            telemetry.addData("USE_CAMERA", USE_CAMERA);
            telemetry.addData("USE_PINPOINT", USE_PINPOINT);

            if (USE_CAMERA) {
                if (hasTarget && turretPose != null) {
                    telemetry.addData("Target found?", true);
                    telemetry.addData("Turret pose (x,y,deg)",
                            "%.1f, %.1f, %.1f",
                            turretPose.getX(), turretPose.getY(), Math.toDegrees(turretPose.getHeading()));
                } else {
                    telemetry.addData("Target found?", false);
                }
            }

            telemetry.addLine("\n=== LOOP (ms) ===");
            telemetry.addData("Loop last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    loopMs, avgLoopMs, minLoopMs, maxLoopMs);
            telemetry.addData("Gap last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    gapMs, avgGapMs, minGapMs, maxGapMs);

            telemetry.addLine("\n=== BULK (ms) ===");
            telemetry.addData("Enabled?", USE_BULK);
            telemetry.addData("Bulk last/avg/min/max (calls=%d)", bulkCount);
            telemetry.addData("Bulk timing",
                    "%.3f / %.3f / %.3f / %.3f",
                    bulkMs, avgBulkMs, minBulkMs, maxBulkMs);

            telemetry.addLine("\n=== COLOR (ms) ===");
            telemetry.addData("Enabled?", USE_COLOR);
            telemetry.addData("Intake last/avg/min/max (calls=%d)", intakeCount);
            telemetry.addData("Intake timing",
                    "%.3f / %.3f / %.3f / %.3f",
                    intakeMs, avgIntakeMs, minIntakeMs, maxIntakeMs);
            telemetry.addData("Feeder last/avg/min/max (calls=%d)", feederCount);
            telemetry.addData("Feeder timing",
                    "%.3f / %.3f / %.3f / %.3f",
                    feederMs, avgFeederMs, minFeederMs, maxFeederMs);

            telemetry.addLine("\n=== PINS (ms) ===");
            telemetry.addData("Enabled?", USE_PINS);
            telemetry.addData("Pins last/avg/min/max (calls=%d)", pinsCount);
            telemetry.addData("Pins timing",
                    "%.3f / %.3f / %.3f / %.3f",
                    pinsMs, avgPinsMs, minPinsMs, maxPinsMs);

            if (USE_CAMERA) {
                telemetry.addLine("\n=== CAMERA (ms) ===");
                telemetry.addData("AutoAim last/avg/min/max (calls=%d)", camCount);
                telemetry.addData("AutoAim timing",
                        "%.3f / %.3f / %.3f / %.3f",
                        camMs, avgCamMs, minCamMs, maxCamMs);
            }

            if (USE_PINPOINT) {
                telemetry.addLine("\n=== PINPOINT (ms) ===");
                telemetry.addData("Pinpoint last/avg/min/max (calls=%d)", pinpointCount);
                telemetry.addData("Pinpoint timing",
                        "%.3f / %.3f / %.3f / %.3f",
                        pinpointMs, avgPinpointMs, minPinpointMs, maxPinpointMs);
            }

            telemetry.addLine();
            telemetry.addData("Runtime (s)", "%.1f", opTimer.seconds());
            telemetry.addData("Loops", loopCount);
            telemetry.addData("Hz (approx)", "%.1f", loopCount / Math.max(0.001, opTimer.seconds()));
            telemetry.addData("Loop Hz (LoopUtil)", "%.1f", LoopUtil.getLoopTimeInHertz());

            telemetry.update();
            Common.dashTelemetry.update();
        }

        if (arducam != null) {
            arducam.close();
        }
    }
}
