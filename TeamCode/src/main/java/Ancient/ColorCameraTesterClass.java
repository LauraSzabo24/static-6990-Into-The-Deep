package Ancient;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Camera.PropDetectorRED;
import TeleOp.ColorDetector;

@Autonomous
@Disabled
public class ColorCameraTesterClass extends LinearOpMode {
    OpenCvCamera cam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        WebcamName camera = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);

        ColorDetector redDetector = new ColorDetector(telemetry);
        cam.setPipeline(redDetector);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                telemetry.addLine("CAMERA WORKS");
                telemetry.update();
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("THE CAMERA DID NOT OPEN PROPERLY SEND HELP", errorCode);
                telemetry.update();
            }
        });

        waitForStart();
        /*switch (redDetector.getColor()) {
            case WHITE:
                telemetry.addLine("white");
                break;
            case PURPLE:
                telemetry.addLine("purple");
                break;
            case YELLOW:
                telemetry.addLine("yellow");
                break;
            case GREEN:
                telemetry.addLine("green");
                break;
            case NOTHING:
                telemetry.addLine("nothing");
        }*/
        telemetry.update();

        //comment out for testing
        cam.stopStreaming();
    }
}
