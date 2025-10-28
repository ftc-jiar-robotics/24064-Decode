package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_MASTERMOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FLYWHEEL_SLAVEMOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.MovingAverageGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.Differentiator;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@Configurable
@Config
public class Flywheel extends Subsystem<Flywheel.FlyWheelStates> {
    private final DcMotorEx shooterMaster, shooterSlave;
    private final DcMotorEx[] motorGroup;

    private final Differentiator differentiator = new Differentiator();

    private final Motor.Encoder shooterEncoder;

    private final PIDController velocityController = new PIDController();

    public static PIDGains shootingVelocityGains = new PIDGains(
            0.000275,
            0.0,
            0.00005,
            Double.POSITIVE_INFINITY
    );

    private final FIRLowPassFilter rpmFilter = new FIRLowPassFilter();
    public static MovingAverageGains rpmDerivAverageFilterGains = new MovingAverageGains(
            10
    );

    private final MovingAverageFilter rpmDerivAverageFilter = new MovingAverageFilter(rpmDerivAverageFilterGains);

    public static LowPassGains rpmFilterGains = new LowPassGains(
            0.3,
            1000
    );

    public enum FlyWheelStates {
        OFF, IDLE, ARMING, RUNNING
    }

    public static double
            RPM_DERIVATIVE_DROP = -2000, // deacceleration
            TIME_DROP_PERIOD = 0.3,
            RPM_TOLERANCE = 50,
            CLOSE_RPM = 3100,
            MOTOR_RPM_SETTLE_TIME = 20,
            IDLE_RPM = 2200,
            MIDDLE_RPM = 3500,
            FAR_RPM = 4100,
            MAX_RPM = 4800;

    public static int[] lutDistances = {0, 90, 130, 160, 180};
    public static int[] lutRPM = {3100, 3500, 3800, 4100, 4800};

    private FlyWheelStates targetState = FlyWheelStates.OFF;

    private int setTargetLoops = 0;

    private boolean inCurrentRPMSpike = false;
    private double
            currentRPM = 0.0,
            currentRPMDerivative = 0.0,
            manualPower = 0.0,
            shootingRPM = 4000,
            lastTarget = shootingRPM,
            currentPower = 0,
            calculatedPower = 0,
            currentRPMSpikeTime = 0;

    public Flywheel(HardwareMap hw) {
        this.shooterMaster = (DcMotorEx) hw.get(DcMotor.class, NAME_FLYWHEEL_MASTERMOTOR);
        this.shooterSlave = (DcMotorEx) hw.get(DcMotor.class, NAME_FLYWHEEL_SLAVEMOTOR);
        MotorEx dummy = new MotorEx(hw, "left front", Motor.GoBILDA.BARE);

        shooterSlave.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMaster.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterEncoder = dummy.encoder;

        motorGroup = new DcMotorEx[]{shooterMaster, shooterSlave};

        velocityController.setGains(shootingVelocityGains);
        rpmFilter.setGains(rpmFilterGains);
    }

    @Override
    public void set(FlyWheelStates f) {
        targetState = f;
    }

    @Override
    public FlyWheelStates get() {
        return targetState;
    }

    public boolean inCurrentSpikeTimer() {
        return inCurrentRPMSpike;
    }

    public void setManualPower(double power) {
        manualPower = power;
    }

    public boolean isPIDInTolerance() {
        return velocityController.isPositionInTolerance(new State(currentRPM, 0, 0, 0), RPM_TOLERANCE);
    }

    public boolean didRPMSpike() {
        currentRPMDerivative = rpmDerivAverageFilter.calculate(differentiator.getDerivative(currentRPM));

        // if current has spiked and we're in tolerance and we're not in a timer(start time + time period > curr time
        if (currentRPMDerivative < RPM_DERIVATIVE_DROP && !inCurrentRPMSpike) {
            currentRPMSpikeTime = (double) System.nanoTime() / 1E9;
            inCurrentRPMSpike = true;
            return true;
        }

        if (currentRPMSpikeTime + TIME_DROP_PERIOD < (double) System.nanoTime() / 1E9 && inCurrentRPMSpike) {
            inCurrentRPMSpike = false;
            return false;
        }

        return false;
    }

    @Override
    public void run() {
        currentRPM = rpmFilter.calculate(shooterEncoder.getCorrectedVelocity() * 60.0 / 28.0);
        if (currentRPM > 10000) currentRPM = 0;

        switch (targetState) {
            case OFF:
                shootingRPM = 0;
                break;
            case IDLE:
                shootingRPM = IDLE_RPM;
                break;
            case ARMING:
                velocityController.setGains(shootingVelocityGains);
                chooseShootingRPM();

                if (isPIDInTolerance()) {
                    targetState = FlyWheelStates.RUNNING;
                }
                break;
            case RUNNING:
                chooseShootingRPM();
                break;
        }

        velocityController.setTarget(new State(shootingRPM, 0, 0, 0));

        if (lastTarget != shootingRPM) {
            currentPower = shootingRPM/MAX_RPM;
            setTargetLoops = LoopUtil.getLoops();
        }

        lastTarget = shootingRPM;

        calculatedPower = velocityController.calculate(new State(currentRPM, 0, 0, 0));

        if (LoopUtil.getLoops() > setTargetLoops + MOTOR_RPM_SETTLE_TIME) currentPower += calculatedPower;

        currentPower = Range.clip(currentPower, 0.0, 1.0);

        for (DcMotorEx m : motorGroup) m.setPower(Math.abs(manualPower) > 0 ? manualPower : currentPower);

        if (isPIDInTolerance()) velocityController.reset();
    }

    private void chooseShootingRPM() {
        shootingRPM = lutRPM[0];
        for (int i = 0; i < lutDistances.length; i++) {
            if (Common.robot.shooter.turret.getDistance() >= lutDistances[i]) shootingRPM = lutRPM[i];
        }
    }


    public void printTelemetry() {
        telemetry.addLine("FLYWHEEL");
        telemetry.addData("current state: ", get());
        telemetry.addData("target state: ", targetState);
        telemetry.addData("current RPM: ", currentRPM);
        telemetry.addData("is PID in tolerance: ", isPIDInTolerance());

        dashTelemetry.addData("current RPM: ", currentRPM);
        dashTelemetry.addData("current RPM Derivative: ", currentRPMDerivative);
        dashTelemetry.addData("calculated power: ", calculatedPower);
        dashTelemetry.addData("current power: ", currentPower);
        dashTelemetry.addData("is timer on: ", inCurrentRPMSpike);
        dashTelemetry.addData("current pos: ", shooterEncoder.getPosition());
        dashTelemetry.addData("target RPM: ", shootingRPM);
        dashTelemetry.addData("target RPM (idle): ", IDLE_RPM);

    }
}
