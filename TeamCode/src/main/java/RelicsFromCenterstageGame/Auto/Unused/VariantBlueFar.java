package RelicsFromCenterstageGame.Auto.Unused;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import RelicsFromCenterstageGame.Auto.Mailbox;
import RelicsFromCenterstageGame.Camera.PropDetectorBLUE;

@Autonomous
@Disabled
public class VariantBlueFar extends LinearOpMode {
    OpenCvCamera cam;
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, intakeMotor;
    private Servo clawServo, armLeftServo, armRightServo, intakeLift;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());

        WebcamName camera = hardwareMap.get(WebcamName.class, "camera");
        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);

        PropDetectorBLUE blueDetector = new PropDetectorBLUE(telemetry);
        cam.setPipeline(blueDetector);
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

        sleep(20);

        //MOTORS AND SERVOS
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "claw");
        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
        intakeLift = hardwareMap.get(Servo.class, "intakeLiftServo");


        //TRAJECTORIES (left/right in robot perspective)
        Pose2d startPose = new Pose2d(-14, 0, Math.toRadians(90)); //90
        drive.setPoseEstimate(startPose);


        //purple pixel
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-10,22,Math.toRadians(120)))
                .strafeLeft(15)
                .forward(5)
                //purple pixel
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeLift.setPosition(0.75);
                        intakeMotor.setPower(-0.7);
                    }
                })
                .addTemporalMarker(3, () -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                    for(int i=0; i<100; i++) {
                        intakeLift.setPosition(0.25);
                    }
                })

                .waitSeconds(1)
                .back(16)
                //yellow pixel
                .lineToLinearHeading(new Pose2d(0,53, Math.toRadians(0)))
                .back(100)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0);
                    }
                })
                .strafeRight(31)

                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(0);
                        armRightServo.setPosition(1);
                    }
                })
                .back(18)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0.3);
                    }
                })
                .forward(10)
                .strafeLeft(31)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(1);
                        armRightServo.setPosition(0);
                    }
                })
                .back(15)
                .build();


        //CENTERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR


        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-5,39,Math.toRadians(180)))
                .waitSeconds(1)
                //purple pixel
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeLift.setPosition(0.75);
                    }
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .waitSeconds(1)
                .addTemporalMarker(3, () -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                    for(int i=0; i<100; i++) {
                        intakeLift.setPosition(0.25);
                    }
                })

                //yellow pixel
                .lineToLinearHeading(new Pose2d(0,53, Math.toRadians(0)))
                .back(100)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0);
                    }
                })
                .strafeRight(24)

                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(0);
                        armRightServo.setPosition(1);
                    }
                })
                .back(18)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0.3);
                    }
                })
                .forward(10)
                .strafeLeft(24)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(1);
                        armRightServo.setPosition(0);
                    }
                })
                .back(15)
                .build();


        //LEFTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-6,16.5,Math.toRadians(90)))
                //purple pixel
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeLift.setPosition(0.75);
                    }
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .addTemporalMarker(3, () -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                    for(int i=0; i<100; i++) {
                        intakeLift.setPosition(0.25);
                    }
                })

                //yellow pixel
                .strafeLeft(19)
                .lineToLinearHeading(new Pose2d(0,53, Math.toRadians(0)))
                .back(100)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0);
                    }
                })
                .strafeRight(20)

                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(0);
                        armRightServo.setPosition(1);
                    }
                })
                .back(18)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0.3);
                    }
                })
                .forward(10)
                .strafeLeft(20)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(1);
                        armRightServo.setPosition(0);
                    }
                })
                .back(15)
                .build();



        waitForStart();
        PropDetectorBLUE.Location place = blueDetector.getLocation();
        telemetry.setMsTransmissionInterval(50);
        if(isStopRequested()) return;

        //DRIVING
        drive.setPoseEstimate(startPose);
        if(place != null) {
            switch (place) {
                case NOT_FOUND:
                    drive.followTrajectorySequence(right);
                    break;
                case CENTER:
                    drive.followTrajectorySequence(left);
                    break;
                case LEFT:
                    drive.followTrajectorySequence(center);

            }
        }
        else{
            drive.followTrajectorySequence(right);
        }

        //mailbox
        Mailbox mail =  new Mailbox();
        mail.setAutoEnd(drive.getPoseEstimate());

    }
}
