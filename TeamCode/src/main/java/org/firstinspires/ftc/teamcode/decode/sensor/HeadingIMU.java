package org.firstinspires.ftc.teamcode.decode.sensor;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.MovingAverageGains;

public final class HeadingIMU {
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private final IMU imu;
    private Thread imuThread;

    private double heading, angularVel;
    private final MovingAverageFilter headingFilter, angularVelFilter;

    public static final int BUFFER_SIZE = 10;
    public static final int STEP_SIZE = 100;
    private int threadLoops = STEP_SIZE;

    public HeadingIMU(HardwareMap hardwareMap, String name, RevHubOrientationOnRobot imuOrientation) {
        synchronized (imuLock) {
            imu = hardwareMap.get(IMU.class, name);
            imu.resetDeviceConfigurationForOpMode();
            imu.resetYaw();
            imu.initialize(new IMU.Parameters(imuOrientation));
        }

        MovingAverageGains gains = new MovingAverageGains(BUFFER_SIZE);
        headingFilter = new MovingAverageFilter(gains);
        angularVelFilter = new MovingAverageFilter(gains);
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                if (threadLoops >= STEP_SIZE) {
                    synchronized (imuLock) {
                        update(BUFFER_SIZE); // Updates values x times (and averages)
                    }
                    threadLoops = 0;
                } else {
                    threadLoops++;
                }
            }
        });
        imuThread.start();
    }

    public void rawUpdate() {
        heading = imu.getRobotYawPitchRollAngles().getYaw(RADIANS);
        angularVel = imu.getRobotAngularVelocity(RADIANS).zRotationRate;
    }

    /**
     * Both values are put into a buffer to automatically be averaged with the last 10 values. This allows us to bypass IMU static and have a greater level of accuracy.
     */
    public void update(int checks) {
        for (int i = 0; i < checks; i++) {
            heading = headingFilter.calculate(imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
            angularVel = angularVelFilter.calculate(imu.getRobotAngularVelocity(RADIANS).zRotationRate);
        }
    }

    public double getHeading() {
        return heading;
    }

    public double getAngularVel() {
        return angularVel;
    }
}
