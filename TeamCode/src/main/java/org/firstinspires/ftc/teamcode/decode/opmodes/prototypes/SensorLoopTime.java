package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_COLOR_SENSOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_LEFT_PIN0;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_RIGHT_PIN0;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_DISTANCE_SENSOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_CAMERA;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.decode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.decode.util.AutoAim;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@TeleOp(name = "SensorLoopTime", group = "Debug")
public class SensorLoopTime extends LinearOpMode {

    private BulkReader bulkReader;

    private ColorSensor intakeColor;
    
    private ColorSensor feederColor;

    private DigitalChannel pin0Left, pin0Right;

    private AutoAim autoAim;

    static final String PINPOINT = "pinpoint";
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        Common.dashTelemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        bulkReader = new BulkReader(hardwareMap);

        // Color sensors
        intakeColor = new ColorSensor(hardwareMap, NAME_INTAKE_DISTANCE_SENSOR, Intake.GAIN);
        feederColor = new ColorSensor(hardwareMap, NAME_FEEDER_COLOR_SENSOR, Feeder.GAIN);

        // LRF digital pins
        pin0Left  = hardwareMap.digitalChannel.get(NAME_FEEDER_LEFT_PIN0);
        pin0Right = hardwareMap.digitalChannel.get(NAME_FEEDER_RIGHT_PIN0);
        pin0Left.setMode(DigitalChannel.Mode.INPUT);
        pin0Right.setMode(DigitalChannel.Mode.INPUT);

        autoAim = new AutoAim(hardwareMap, NAME_TURRET_CAMERA);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);

        telemetry.addLine("Sensor Loop Time (Color + Pins + Cam + Pinpoint)");
        telemetry.addData("Intake Color", NAME_INTAKE_DISTANCE_SENSOR);
        telemetry.addData("Feeder Color", NAME_FEEDER_COLOR_SENSOR);
        telemetry.addData("Left Pin0", NAME_FEEDER_LEFT_PIN0);
        telemetry.addData("Right Pin0", NAME_FEEDER_RIGHT_PIN0);
        telemetry.addData("Camera", NAME_TURRET_CAMERA);
        telemetry.addData("Pinpoint", PINPOINT);
        telemetry.update();

        waitForStart();

        ElapsedTime opTimer = new ElapsedTime();
        opTimer.reset();

        long lastLoopStartNs = System.nanoTime();
        int loopCount = 0;

        // --- Stats containers ---
        double minLoopMs = Double.POSITIVE_INFINITY, maxLoopMs = 0, sumLoopMs = 0;
        double minGapMs  = Double.POSITIVE_INFINITY, maxGapMs  = 0, sumGapMs  = 0;

        double minBulkMs = Double.POSITIVE_INFINITY, maxBulkMs = 0, sumBulkMs = 0;
        double minIntakeMs = Double.POSITIVE_INFINITY, maxIntakeMs = 0, sumIntakeMs = 0;
        double minFeederMs = Double.POSITIVE_INFINITY, maxFeederMs = 0, sumFeederMs = 0;
        double minPinsMs = Double.POSITIVE_INFINITY, maxPinsMs = 0, sumPinsMs = 0;

        double minCamMs = Double.POSITIVE_INFINITY, maxCamMs = 0, sumCamMs = 0;
        double minPinpointMs = Double.POSITIVE_INFINITY, maxPinpointMs = 0, sumPinpointMs = 0;

        while (opModeIsActive()) {

            LoopUtil.updateLoopCount();

            long loopStartNs = System.nanoTime();
            double gapMs = (loopStartNs - lastLoopStartNs) / 1e6;
            lastLoopStartNs = loopStartNs;

            // -------- Bulk read timing --------
            long bulkStartNs = System.nanoTime();
            bulkReader.bulkRead();
            long bulkEndNs = System.nanoTime();
            double bulkMs = (bulkEndNs - bulkStartNs) / 1e6;

            // -------- Color sensor timing --------
            long intakeStartNs = System.nanoTime();
            intakeColor.update();
            long intakeEndNs = System.nanoTime();
            double intakeMs = (intakeEndNs - intakeStartNs) / 1e6;

            long feederStartNs = System.nanoTime();
            feederColor.update();
            long feederEndNs = System.nanoTime();
            double feederMs = (feederEndNs - feederStartNs) / 1e6;

            // -------- LRF digital pin timing --------
            long pinsStartNs = System.nanoTime();
            boolean leftState = pin0Left.getState();
            boolean rightState = pin0Right.getState();
            long pinsEndNs = System.nanoTime();
            double pinsMs = (pinsEndNs - pinsStartNs) / 1e6;

            // -------- Camera / AutoAim timing --------
            long camStartNs = System.nanoTime();
            boolean hasTarget = autoAim.detectTarget();
            Pose turretPose = null;
            if (hasTarget) {
                turretPose = autoAim.getTurretPosePedro();
            }
            long camEndNs = System.nanoTime();
            double camMs = (camEndNs - camStartNs) / 1e6;

            // -------- Pinpoint timing --------
            long ppStartNs = System.nanoTime();
            pinpoint.update();
            long ppEndNs = System.nanoTime();
            double pinpointMs = (ppEndNs - ppStartNs) / 1e6;

            // -------- Total loop timing --------
            long loopEndNs = System.nanoTime();
            double loopMs = (loopEndNs - loopStartNs) / 1e6;

            loopCount++;

            // --- Stats: loop & gap ---
            minLoopMs = Math.min(minLoopMs, loopMs);
            maxLoopMs = Math.max(maxLoopMs, loopMs);
            sumLoopMs += loopMs;
            double avgLoopMs = sumLoopMs / loopCount;

            minGapMs = Math.min(minGapMs, gapMs);
            maxGapMs = Math.max(maxGapMs, gapMs);
            sumGapMs += gapMs;
            double avgGapMs = sumGapMs / loopCount;

            // --- Stats: bulk ---
            minBulkMs = Math.min(minBulkMs, bulkMs);
            maxBulkMs = Math.max(maxBulkMs, bulkMs);
            sumBulkMs += bulkMs;
            double avgBulkMs = sumBulkMs / loopCount;

            // --- Stats: color ---
            minIntakeMs = Math.min(minIntakeMs, intakeMs);
            maxIntakeMs = Math.max(maxIntakeMs, intakeMs);
            sumIntakeMs += intakeMs;
            double avgIntakeMs = sumIntakeMs / loopCount;

            minFeederMs = Math.min(minFeederMs, feederMs);
            maxFeederMs = Math.max(maxFeederMs, feederMs);
            sumFeederMs += feederMs;
            double avgFeederMs = sumFeederMs / loopCount;

            // --- Stats: pins ---
            minPinsMs = Math.min(minPinsMs, pinsMs);
            maxPinsMs = Math.max(maxPinsMs, pinsMs);
            sumPinsMs += pinsMs;
            double avgPinsMs = sumPinsMs / loopCount;

            // --- Stats: camera ---
            minCamMs = Math.min(minCamMs, camMs);
            maxCamMs = Math.max(maxCamMs, camMs);
            sumCamMs += camMs;
            double avgCamMs = sumCamMs / loopCount;

            // --- Stats: pinpoint ---
            minPinpointMs = Math.min(minPinpointMs, pinpointMs);
            maxPinpointMs = Math.max(maxPinpointMs, pinpointMs);
            sumPinpointMs += pinpointMs;
            double avgPinpointMs = sumPinpointMs / loopCount;


            if (hasTarget && turretPose != null) {
                telemetry.addData("Target found?", true);
                telemetry.addData("Turret pose (x,y,deg)",
                        "%.1f, %.1f, %.1f",
                        turretPose.getX(), turretPose.getY(), Math.toDegrees(turretPose.getHeading()));
            } else {
                telemetry.addData("Target found?", false);
            }

            // -------- Telemetry: timing --------
            telemetry.addLine("\n=== LOOP (ms) ===");
            telemetry.addData("Loop last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    loopMs, avgLoopMs, minLoopMs, maxLoopMs);
            telemetry.addData("Gap last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    gapMs, avgGapMs, minGapMs, maxGapMs);

            telemetry.addLine("\n=== BULK (ms) ===");
            telemetry.addData("Bulk last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    bulkMs, avgBulkMs, minBulkMs, maxBulkMs);

            telemetry.addLine("\n=== COLOR (ms) ===");
            telemetry.addData("Intake last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    intakeMs, avgIntakeMs, minIntakeMs, maxIntakeMs);
            telemetry.addData("Feeder last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    feederMs, avgFeederMs, minFeederMs, maxFeederMs);

            telemetry.addLine("\n=== PINS (ms) ===");
            telemetry.addData("Pins last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    pinsMs, avgPinsMs, minPinsMs, maxPinsMs);

            telemetry.addLine("\n=== CAMERA (ms) ===");
            telemetry.addData("AutoAim.detectTarget last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    camMs, avgCamMs, minCamMs, maxCamMs);

            telemetry.addLine("\n=== PINPOINT (ms) ===");
            telemetry.addData("Pinpoint.update last/avg/min/max",
                    "%.3f / %.3f / %.3f / %.3f",
                    pinpointMs, avgPinpointMs, minPinpointMs, maxPinpointMs);

            telemetry.addLine();
            telemetry.addData("Runtime (s)", "%.1f", opTimer.seconds());
            telemetry.addData("Loops", loopCount);
            telemetry.addData("Hz (approx)", "%.1f", loopCount / opTimer.seconds());
            telemetry.addData("Loop Hz (LoopUtil)", "%.1f", LoopUtil.getLoopTimeInHertz());

            telemetry.update();
            Common.dashTelemetry.update();
        }

        if (autoAim != null) {
            autoAim.close();
        }
    }
}
