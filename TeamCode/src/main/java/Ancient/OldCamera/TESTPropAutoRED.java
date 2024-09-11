package Ancient.OldCamera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;

import Camera.PropDetectorRED;

@Autonomous
@Disabled
public class TESTPropAutoRED extends LinearOpMode {
    OpenCvCamera cam;
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, intakeMotor;
    private Servo intakeLiftServo;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        WebcamName camera = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);

        PropDetectorRED redDetector = new PropDetectorRED(telemetry);
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

        //drive motors
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
        intakeLiftServo = hardwareMap.get(Servo.class, "intakeLiftServo");

        waitForStart();
        switch (redDetector.getLocation()) {
            case NOT_FOUND:
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(0.5);
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(-0.5);
                sleep(1200);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                sleep(500);
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(-0.5);
                sleep(200);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                intakeLiftServo.setPosition(1);
                intakeMotor.setPower(-0.8);
                sleep(1000);
                intakeMotor.setPower(0);
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(0.5);
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(-0.5);
                sleep(3000);
                break;
            case CENTER:
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(-0.5);
                sleep(800);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                intakeLiftServo.setPosition(1);
                intakeMotor.setPower(-0.8);
                sleep(1000);
                intakeMotor.setPower(0);
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(0.5);
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(-0.5);
                sleep(3000);
                break;
            case LEFT:
                motorFrontLeft.setPower(0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(0.5);
                sleep(800);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                sleep(500);
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(-0.5);
                motorFrontRight.setPower(-0.5);
                motorBackRight.setPower(-0.5);
                sleep(400);
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
                intakeLiftServo.setPosition(1);
                intakeMotor.setPower(-0.8);
                sleep(1000);
                intakeMotor.setPower(0);
                motorFrontLeft.setPower(-0.5);
                motorBackLeft.setPower(0.5);
                motorFrontRight.setPower(0.5);
                motorBackRight.setPower(-0.5);
                sleep(3000);
        }
        //comment out for testing
        cam.stopStreaming();

        //do the rest of the auto
    }
}
