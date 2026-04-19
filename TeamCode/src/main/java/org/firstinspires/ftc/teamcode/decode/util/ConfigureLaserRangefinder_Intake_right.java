package org.firstinspires.ftc.teamcode.decode.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;

@Autonomous
public class ConfigureLaserRangefinder_Intake_right extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LaserRangefinder leftLRF = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "intakeFront"));
        telemetry.addLine("LEFT LRF\n");
        telemetry.addData("Pin0", leftLRF.getPin0Mode());
        telemetry.addData("Pin1", leftLRF.getPin1Mode());
        telemetry.addData("Distance Mode", leftLRF.getDistanceMode().name());
        telemetry.addData("Timing [Budget, Period]", java.util.Arrays.toString(leftLRF.getTiming()));
        telemetry.addData("ROI", java.util.Arrays.toString(leftLRF.getROI()));
        telemetry.addData("Optical Center", java.util.Arrays.toString(leftLRF.getOpticalCenter()));
        telemetry.update();
        waitForStart();
        leftLRF.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        leftLRF.setTiming(10, 0);
        leftLRF.setROI(0, 15, 10, 0);
        leftLRF.setPin0Digital(Common.MIN_DISTANCE_FEEDER, 100);
        leftLRF.setPin1Digital(Common.MIN_DISTANCE_FEEDER, 100);

    }
}

