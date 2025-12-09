package org.firstinspires.ftc.teamcode.decode.util;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightEx {
    private LLResult result;
    private final Limelight3A limelight;

    public LimelightEx(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public LLResult update() {
        result = limelight.getLatestResult();
        return result;
    }

    public LLResult getResult() {
        return result;
    }

    public List<LLResultTypes.ColorResult> getColorResult() {
        return result.getColorResults();
    }

    public List<LLResultTypes.DetectorResult> getDetectorResult() {
        if (result != null) return result.getDetectorResults();
        return null;
    }

    public Pose getPoseEstimate() {
        Pose pose = null;
        if (result != null && result.isValid()) {
            Pose3D pose3D = result.getBotpose_MT2();
            if (pose3D != null) {
                pose = new Pose(pose3D.getPosition().x, pose3D.getPosition().y, 0);
            }
        }
        return pose;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }
}
