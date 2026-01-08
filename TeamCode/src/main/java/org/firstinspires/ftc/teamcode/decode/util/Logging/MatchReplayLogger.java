package org.firstinspires.ftc.teamcode.decode.util.Logging;

import android.annotation.SuppressLint;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * MatchReplayLogger
 * - One file per match
 * - Auto writes first, TeleOp appends to same file
 * - Uses a small session file so TeleOp can find the active match file
 * - Buffered async flush to avoid killing loop time
 */
public final class MatchReplayLogger {

    public enum Phase { AUTO, TELEOP }

    private static final String DIR_NAME = "matchlogs";
    private static final String SESSION_FILE_NAME = "active_match_session.txt";

    // Tuning
    private static final int FLUSH_LINE_THRESHOLD = 120;
    private static final long FLUSH_TIME_MS = 250;
    private static final int MAX_QUEUE_LINES = 6000;

    private static MatchReplayLogger INSTANCE;

    public static synchronized MatchReplayLogger get() {
        if (INSTANCE == null) INSTANCE = new MatchReplayLogger();
        return INSTANCE;
    }

    private final ExecutorService ioThread = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "MatchReplayLogger-IO");
        t.setDaemon(true);
        return t;
    });

    private final ArrayDeque<String> queue = new ArrayDeque<>(4096);

    private BufferedWriter writer;
    private File logFile;

    private String matchId = "";
    private String autoOp = "";
    private String teleOp = "";
    private Phase phase = Phase.AUTO;

    private boolean started = false;

    private long lastFlushMs = 0;
    private int queuedSinceFlush = 0;

    private double lastRuntimeSec = Double.NaN;
    private int loopCounter = 0;

    private MatchReplayLogger() {}

    public void logRobot(Robot robot, double runtimeSec) {
        Pose p = robot.drivetrain.getPose();

        logSnapshot(
                runtimeSec,
                p.getX(), p.getY(), p.getHeading(),
                robot.drivetrain.getVelocity().getXComponent(),
                robot.drivetrain.getVelocity().getYComponent(),
                robot.drivetrain.getAngularVelocity(),
                robot.isRobotMoving(),
                Common.inTriangle,
                robot.batteryVoltageSensor.getVoltage(),
                robot.shooter.getQueuedShots(),
                robot.shooter.getFlywheelRpm(),
                robot.shooter.getFlywheelError(),
                robot.shooter.getTurretAngle(),
                robot.shooter.getTurretError(),
                robot.shooter.getHoodTargetDeg(),
                robot.shooter.isBallPresent()
        );
    }


    // ---------------------------
    // Public API
    // ---------------------------

    /** Call once at the start of your Autonomous opmode (after waitForStart). */
    public synchronized File startAutonomous(String autonomousOpModeName) {
        phase = Phase.AUTO;
        autoOp = autonomousOpModeName != null ? autonomousOpModeName : "UNKNOWN_AUTO";
        teleOp = "";

        // New match id each time auto starts
        matchId = timestampId();
        logFile = new File(getLogDir(), "match-" + matchId + ".csv");

        openWriter(logFile, false);
        writeHeaderIfNeeded();
        writeMetaLine("START_AUTO", "auto=" + autoOp);

        writeSessionFile(logFile, matchId, autoOp);

        started = true;
        lastRuntimeSec = Double.NaN;
        loopCounter = 0;

        return logFile;
    }

    /** Call once at the start of your TeleOp opmode (after waitForStart). */
    public synchronized File startTeleOp(String teleOpModeName) {
        phase = Phase.TELEOP;
        teleOp = teleOpModeName != null ? teleOpModeName : "UNKNOWN_TELE";

        // Try to append to the last auto match file
        Session s = readSessionFile();
        if (s != null && s.file != null && s.file.exists()) {
            logFile = s.file;
            matchId = s.matchId;
            autoOp = s.autoOp;
            openWriter(logFile, true);
            writeMetaLine("START_TELEOP", "auto=" + autoOp + ", tele=" + teleOp);
        } else {
            // TeleOp started without a prior auto (testing)
            matchId = timestampId();
            logFile = new File(getLogDir(), "match-" + matchId + ".csv");
            openWriter(logFile, false);
            writeHeaderIfNeeded();
            writeMetaLine("START_TELEOP", "tele=" + teleOp);
        }

        started = true;
        lastRuntimeSec = Double.NaN;
        loopCounter = 0;

        return logFile;
    }

    /** Call every loop. Keep it lightweight: just pass numbers and strings. */
    public void logSnapshot(
            double runtimeSec,
            // pose
            double x, double y, double headingRad,
            // drivetrain
            double vx, double vy, double omega,
            // status
            boolean robotMoving,
            boolean inTriangle,
            double voltage,
            // shooter-ish
            int queuedShots,
            double flywheelRpm,
            double flywheelErr,
            double turretDeg,
            double turretErr,
            double hoodTargetDeg,
            boolean ballPresent
    ) {
        if (!started) return;

        loopCounter++;

        double dt = Double.isNaN(lastRuntimeSec) ? 0.0 : (runtimeSec - lastRuntimeSec);
        lastRuntimeSec = runtimeSec;

        // type=SAMPLE so you can filter in analysis
        String line =
                matchId + "," +
                        phase.name() + "," +
                        safe(autoOp) + "," +
                        safe(teleOp) + "," +
                        "SAMPLE" + "," +
                        "" + "," +                      // label
                        runtimeSec + "," +
                        dt + "," +
                        loopCounter + "," +
                        x + "," + y + "," + headingRad + "," +
                        vx + "," + vy + "," + omega + "," +
                        (robotMoving ? 1 : 0) + "," +
                        (inTriangle ? 1 : 0) + "," +
                        voltage + "," +
                        queuedShots + "," +
                        flywheelRpm + "," +
                        flywheelErr + "," +
                        turretDeg + "," +
                        turretErr + "," +
                        hoodTargetDeg + "," +
                        (ballPresent ? 1 : 0) + "," +
                        "" +                            // message
                        "\n";

        enqueue(line);
    }

    /** Use this for labeled checkpoints: "START_SHOOT_PRELOAD", "SHOT_FIRED", etc. */
    public void event(double runtimeSec, String label, String message) {
        if (!started) return;

        // type=EVENT
        String line =
                matchId + "," +
                        phase.name() + "," +
                        safe(autoOp) + "," +
                        safe(teleOp) + "," +
                        "EVENT" + "," +
                        safe(label) + "," +
                        runtimeSec + "," +
                        "" + "," +                      // dt blank
                        loopCounter + "," +
                        "" + "," + "" + "," + "" + "," + // pose blank
                        "" + "," + "" + "," + "" + "," + // vel blank
                        "" + "," + "" + "," +            // flags blank
                        "" + "," +                       // voltage blank
                        "" + "," +                       // queuedShots blank
                        "" + "," + "" + "," +            // fly blank
                        "" + "," + "" + "," +            // turret blank
                        "" + "," +                       // hood blank
                        "" + "," +                       // ball blank
                        safe(message) +
                        "\n";

        enqueue(line);
        flushAsync(); // events flush faster
    }

    /** Call at the end of Auto opmode. Leaves session file so TeleOp can append. */
    public synchronized void stopAutonomous(double runtimeSec) {
        if (!started) return;
        writeMetaLine("END_AUTO", "t=" + runtimeSec);
        closeWriterAsync();
        started = false;
        // DO NOT delete session file here
    }

    /** Call at the end of TeleOp opmode. Deletes session file (match is done). */
    public synchronized void stopTeleOp(double runtimeSec) {
        if (!started) return;
        writeMetaLine("END_TELEOP", "t=" + runtimeSec);
        closeWriterAsync();
        started = false;
        deleteSessionFile();
    }

    public synchronized File getLogFile() {
        return logFile;
    }

    // ---------------------------
    // Internal helpers
    // ---------------------------

    private File getLogDir() {
        File dir = new File(AppUtil.ROBOT_DATA_DIR, DIR_NAME);
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        return dir;
    }

    private File getSessionFile() {
        return new File(getLogDir(), SESSION_FILE_NAME);
    }

    private void writeHeaderIfNeeded() {
        // CSV header
        // matchId,phase,autoOp,teleOp,type,label,runtimeSec,dtSec,loop,x,y,headingRad,vx,vy,omega,robotMoving,inTriangle,voltage,queuedShots,flywheelRpm,flywheelErr,turretDeg,turretErr,hoodTargetDeg,ballPresent,message
        try {
            writer.write("matchId,phase,autoOp,teleOp,type,label,runtimeSec,dtSec,loop,x,y,headingRad,vx,vy,omega,robotMoving,inTriangle,voltage,queuedShots,flywheelRpm,flywheelErr,turretDeg,turretErr,hoodTargetDeg,ballPresent,message\n");
            writer.flush();
        } catch (IOException ignored) {}
    }

    private void writeMetaLine(String label, String msg) {
        event(0.0, label, msg); // runtimeSec may not be meaningful here, but label matters
    }

    private void openWriter(File file, boolean append) {
        try {
            writer = new BufferedWriter(new FileWriter(file, append), 64 * 1024);
            lastFlushMs = System.currentTimeMillis();
            queuedSinceFlush = 0;
            queue.clear();
        } catch (IOException e) {
            writer = null;
        }
    }

    private void enqueue(String line) {
        synchronized (queue) {
            if (queue.size() >= MAX_QUEUE_LINES) queue.pollFirst();
            queue.addLast(line);
        }

        queuedSinceFlush++;
        long now = System.currentTimeMillis();
        if (queuedSinceFlush >= FLUSH_LINE_THRESHOLD || (now - lastFlushMs) >= FLUSH_TIME_MS) {
            queuedSinceFlush = 0;
            lastFlushMs = now;
            flushAsync();
        }
    }

    private void flushAsync() {
        ioThread.execute(() -> {
            BufferedWriter w;
            synchronized (this) { w = writer; }
            if (w == null) return;

            StringBuilder sb = new StringBuilder(64 * 1024);
            synchronized (queue) {
                while (!queue.isEmpty() && sb.length() < 512 * 1024) {
                    sb.append(queue.pollFirst());
                }
            }

            if (sb.length() == 0) return;

            try {
                w.write(sb.toString());
                w.flush();
            } catch (IOException ignored) {}
        });
    }

    private void closeWriterAsync() {
        flushAsync();
        ioThread.execute(() -> {
            BufferedWriter w;
            synchronized (this) { w = writer; writer = null; }
            if (w == null) return;
            try {
                w.flush();
                w.close();
            } catch (IOException ignored) {}
        });
    }

    private void writeSessionFile(File file, String matchId, String autoOp) {
        File s = getSessionFile();
        try (BufferedWriter bw = new BufferedWriter(new FileWriter(s, false))) {
            bw.write(file.getAbsolutePath() + "\n");
            bw.write(matchId + "\n");
            bw.write(autoOp + "\n");
        } catch (IOException ignored) {}
    }

    private Session readSessionFile() {
        File s = getSessionFile();
        if (!s.exists()) return null;

        try (BufferedReader br = new BufferedReader(new FileReader(s))) {
            String path = br.readLine();
            String mid = br.readLine();
            String aop = br.readLine();
            if (path == null || mid == null) return null;
            Session out = new Session();
            out.file = new File(path);
            out.matchId = mid;
            out.autoOp = (aop == null) ? "" : aop;
            return out;
        } catch (IOException e) {
            return null;
        }
    }

    private void deleteSessionFile() {
        File s = getSessionFile();
        //noinspection ResultOfMethodCallIgnored
        s.delete();
    }

    @SuppressLint("SimpleDateFormat")
    private String timestampId() {
        return new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
    }

    private String safe(String s) {
        if (s == null) return "";
        // keep CSV stable
        return s.replace(",", ";").replace("\n", " ").replace("\r", " ");
    }

    private static final class Session {
        File file;
        String matchId;
        String autoOp;
    }
}
