package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_COLOR_SENSOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_LEFT_PIN0;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_RIGHT_PIN0;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_DISTANCE_SENSOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.decode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@TeleOp(name = "SensorLoopTime_ColorDistance", group = "Debug")
public class SensorLoopTime_ColorDistance extends LinearOpMode {

    private BulkReader bulkReader;

    private ColorSensor intakeColor;
    private ColorSensor feederColor;

    private DigitalChannel pin0Left, pin0Right;

    @Override
    public void runOpMode() throws InterruptedException {

        // Combined DS + Dashboard telemetry
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

        telemetry.addLine("Sensor Loop Time (Color + Distance Pins)");
        telemetry.addData("Intake Color", NAME_INTAKE_DISTANCE_SENSOR);
        telemetry.addData("Feeder Color", NAME_FEEDER_COLOR_SENSOR);
        telemetry.addData("Left Pin0", NAME_FEEDER_LEFT_PIN0);
        telemetry.addData("Right Pin0", NAME_FEEDER_RIGHT_PIN0);
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

            telemetry.addLine();
            telemetry.addData("Runtime (s)", "%.1f", opTimer.seconds());
            telemetry.addData("Loops", loopCount);
            telemetry.addData("Hz (approx)", "%.1f", loopCount / opTimer.seconds());
            telemetry.addData("Loop Hz (LoopUtil)", "%.1f", LoopUtil.getLoopTimeInHertz());

            telemetry.update();
            Common.dashTelemetry.update();
        }
    }
}
