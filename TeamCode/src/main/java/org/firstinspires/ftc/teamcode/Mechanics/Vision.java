package org.firstinspires.ftc.teamcode.Mechanics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision {
    public String motif = "";
    private static AprilTagProcessor aprilTag;
    private static VisionPortal visionPortal;
    private final boolean USE_WEBCAM = true;
    public void setManualExposure(int exposureMS, int gain, Telemetry telemetry) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                Thread.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            Thread.sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        Thread.sleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        Thread.sleep(20);
    }
    public void initAprilTag(HardwareMap hardwareMap) {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
//
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    public String findMotif() {
        if (motif.isEmpty()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                switch (detection.id) {
                    case 21:
                        motif = "GPP";
                        visionPortal.stopStreaming();
                        break;
                    case 22:
                        motif = "PGP";
                        visionPortal.stopStreaming();
                        break;
                    case 23:
                        motif = "PPG";
                        visionPortal.stopStreaming();
                        break;
                    default:
                        telemetry.addData("Skipping", "this sigma is NOT the obelisk", detection.id);
                }
            }
        }
        return motif;
    }
}
