package org.firstinspires.ftc.teamcode.decode.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;

@Autonomous
public class ConfigureLaserRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LaserRangefinder leftLRF = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, Common.NAME_FEEDER_LEFT_DISTANCE_SENSOR));
        LaserRangefinder rightLRF = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, Common.NAME_FEEDER_RIGHT_DISTANCE_SENSOR));
        telemetry.addLine("LEFT LRF\n");
        telemetry.addData("Pin0", leftLRF.getPin0Mode());
        telemetry.addData("Pin1", leftLRF.getPin1Mode());
        telemetry.addData("Distance Mode", leftLRF.getDistanceMode().name());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(leftLRF.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(leftLRF.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(leftLRF.getOpticalCenter()));
        telemetry.addLine("RIGHT LRF\n");
        telemetry.addData("Pin0", rightLRF.getPin0Mode());
        telemetry.addData("Pin1", rightLRF.getPin1Mode());
        telemetry.addData("Distance Mode", rightLRF.getDistanceMode().name());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(rightLRF.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(rightLRF.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(rightLRF.getOpticalCenter()));
        telemetry.update();
        waitForStart();
        leftLRF.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        rightLRF.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        leftLRF.setTiming(10, 0);
        rightLRF.setTiming(10, 0);
        leftLRF.setROI(0, 10, 15, 0);
        rightLRF.setROI(0, 10, 15, 0);
        leftLRF.setPin0Digital(Common.MIN_DISTANCE_FEEDER, Common.MAX_DISTANCE_FEEDER);
        leftLRF.setPin1Digital(Common.MIN_DISTANCE_FEEDER, Common.MAX_DISTANCE_FEEDER);
        rightLRF.setPin0Digital(Common.MIN_DISTANCE_FEEDER, Common.MAX_DISTANCE_FEEDER);
        rightLRF.setPin1Digital(Common.MIN_DISTANCE_FEEDER, Common.MAX_DISTANCE_FEEDER);

    }
}

