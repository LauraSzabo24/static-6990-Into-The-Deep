package Ancient.OldCamera;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import Camera.AprilTagDetectionPipeline;

import java.util.ArrayList;
import java.util.List;


@Autonomous
@Disabled
public class PropRecognitionTest1 extends LinearOpMode
{

    //TAG THINGS
    private AprilTagDetectionPipeline tagPipe;
    private OpenCvCamera cam;
    static final double FEET_PER_METER = 3.28084;
    private static final double fx = 578.272;
    private static final double fy = 578.272;
    private static final double cx = 402.145;
    private static final double cy = 221.506;
    // units in meters
    private static final double tagsize = 0.166;

    private int numFramesWithoutDetection = 0;
    private int numDetectionCycle = 0;

    private static final float DECIMATION_HIGH = 3;
    private static final float DECIMATION_LOW = 2;
    private static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    private static final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    private static int tagNumber;

    //TENSORFLOW THINGS
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private static final String TFOD_MODEL_ASSET = "gomba.tflite";
    private static final String[] LABELS = {
            "labels.txt",
    };

    @Override
    public void runOpMode()
    {
/*
        //TAG THINGS
        tagNumber = 0;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        tagPipe = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        cam.setPipeline(tagPipe);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }
            @Override
            public void onError(int errorCode)
            {
                throw new RuntimeException("camera not working" + errorCode);
            }
        });
        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive() && !(tagNumber == 1) && !(tagNumber == 2) && !(tagNumber == 3) && !(tagNumber == 4)) {
            telemetry.addData("we're in the loop", cam.getFps());
            ArrayList<AprilTagDetection> detections = tagPipe.getDetectionsUpdate();

            if (detections != null) {
                telemetry.addData("FPS", cam.getFps());
                telemetry.addData("Overhead ms", cam.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", cam.getPipelineTimeMs());

                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        tagPipe.setDecimation(DECIMATION_LOW);
                    }
                } else {
                    numFramesWithoutDetection = 0;

                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        tagPipe.setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        /*telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        */
        /*
                        //...
                        if (detection.id == 1 || detection.id == 2 || detection.id == 3) {
                            tagNumber = detection.id;
                            telemetry.addLine(String.format("\nfinished tags", tagNumber));
                            telemetry.update();
                        }                        //...
                    }
                }
                telemetry.update();
            }

            sleep(20);

        }
        cam.stopStreaming();

*/


        //TENSORFLOW THINGS
        initTfod();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                //telemetry.addLine(String.format("\nstarted object recognition", tagNumber));
                telemetryTfod();
                telemetry.update();
                sleep(20);
            }
        }
        visionPortal.close();


        //TRAJECTORY THINGS
    }
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)

                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.setCameraResolution(new Size(640, 480));
        //builder.enableCameraMonitoring(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);

        builder.addProcessor(tfod);
        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.5f);

        //not sure
        visionPortal.setProcessorEnabled(tfod, true);
    }
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}
