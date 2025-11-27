package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import android.graphics.Bitmap;
import android.os.Environment;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileOutputStream;

@TeleOp(name="Camera Calibration Capture", group="Calibration")
public class CalibrationCapture extends LinearOpMode {

    OpenCvCamera camera;
    CalibrationPipeline pipeline;
    int savedCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        int camMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);

        pipeline = new CalibrationPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(() -> {
            camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        });

        telemetry.addLine("Press PLAY to start. Hold a checkerboard in view.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Press A to save current frame");
            telemetry.addLine("Total saved: " + savedCount);
            telemetry.update();

            if (gamepad1.a) {
                saveFrame(pipeline.lastFrame);
                savedCount++;
                sleep(400);
            }

            sleep(10);
        }
    }

    private void saveFrame(Mat input) {
        try {
            Bitmap bmp = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(input, bmp);

            File folder = new File(Environment.getExternalStorageDirectory(), "calibration");
            if (!folder.exists()) folder.mkdirs();

            File file = new File(folder, "frame_" + savedCount + ".png");
            FileOutputStream out = new FileOutputStream(file);
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.close();
        }
        catch (Exception e) {
            telemetry.addData("ERROR saving frame", e.toString());
        }
    }

    // Pass-through pipeline that stores the current frame
    public static class CalibrationPipeline extends OpenCvPipeline {
        public Mat lastFrame = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(lastFrame);
            return input;
        }
    }
}

