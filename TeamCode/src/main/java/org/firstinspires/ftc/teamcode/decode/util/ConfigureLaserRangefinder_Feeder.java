package org.firstinspires.ftc.teamcode.decode.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;

@Autonomous
public class ConfigureLaserRangefinder_Feeder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LaserRangefinder rightLRF = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "intakeSensorFeeder"));
        telemetry.addLine("RIGHT LRF\n");
        telemetry.addData("Pin0", rightLRF.getPin0Mode());
        telemetry.addData("Pin1", rightLRF.getPin1Mode());
        telemetry.addData("Distance Mode", rightLRF.getDistanceMode().name());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(rightLRF.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(rightLRF.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(rightLRF.getOpticalCenter()));
        telemetry.update();
        waitForStart();
        rightLRF.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        rightLRF.setTiming(10, 0);
        rightLRF.setROI(0, 15, 15, 0);
        rightLRF.setPin0Digital(Common.MIN_DISTANCE_FEEDER, 100);
        rightLRF.setPin1Digital(Common.MIN_DISTANCE_FEEDER, 100);

    }
}

