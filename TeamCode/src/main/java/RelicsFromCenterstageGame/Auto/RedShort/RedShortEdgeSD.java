package RelicsFromCenterstageGame.Auto.RedShort;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.MecanumDrives.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import RelicsFromCenterstageGame.Auto.Mailbox;
import RelicsFromCenterstageGame.Camera.PropDetectorRED;

@Autonomous
public class RedShortEdgeSD extends LinearOpMode {
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

        sleep(20);

        // region MOTORS AND SERVOS
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "claw");
        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
        intakeLift = hardwareMap.get(Servo.class, "intakeLiftServo");
        //endregion

        //region TRAJECTORIES (left/right in robot perspective)
        Pose2d startPose = new Pose2d(-14, 0, Math.toRadians(-90)); //90
        drive.setPoseEstimate(startPose);
        //endregion

        //region RIGHT
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-9,-23,Math.toRadians(-60))) //-10, -22 -120
                .strafeLeft(10) //right
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
                .back(16) //16
                //yellow pixel
                .lineToLinearHeading(new Pose2d(-30,-53, Math.toRadians(0)))
                .back(10)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0);
                    }
                })
                .strafeLeft(20)

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
                .waitSeconds(1)
                .forward(10)
                .strafeLeft(37)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(1);
                        armRightServo.setPosition(0);
                    }
                })
                .back(15)
                .build();
        //endregion

        //region CENTER
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-22,-39,Math.toRadians(0))) //-25 -39
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
                .lineToLinearHeading(new Pose2d(-30,-53, Math.toRadians(0)))
                .back(10)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0);
                    }
                })
                .strafeLeft(25)

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
                .waitSeconds(1)
                .forward(10)
                .strafeLeft(28)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(1);
                        armRightServo.setPosition(0);
                    }
                })
                .back(15)
                .build();
        //endregion

        //region LEFT
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-16.4,-16.5,Math.toRadians(-90))) //-21
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
                .strafeRight(16)
                .lineToLinearHeading(new Pose2d(-30,-53, Math.toRadians(0)))
                .back(10)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0);
                    }
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                })
                .strafeLeft(33)

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
                .waitSeconds(1)
                .forward(10)
                .strafeLeft(25)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        armLeftServo.setPosition(1);
                        armRightServo.setPosition(0);
                    }
                })
                .back(15)
                .build();
        //endregion

        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose) //(-14, 0_)
                .turn(Math.toRadians(123))
                .build();

        waitForStart();
        PropDetectorRED.Location place = redDetector.getLocation();
        telemetry.setMsTransmissionInterval(50);
        if(isStopRequested()) return;
        Mailbox mail = new Mailbox();

        //DRIVING
        drive.setPoseEstimate(startPose);
        if(place != null) {
            switch (place) {
                case NOT_FOUND:
                    drive.followTrajectorySequence(right, mail);
                    break;
                case CENTER:
                    drive.followTrajectorySequence(left, mail);
                    break;
                case LEFT:
                    drive.followTrajectorySequence(center, mail);

            }
        }
        else{
            drive.followTrajectorySequence(right, mail);
        }
        //drive.followTrajectorySequence(test, mail);

        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(-180))));
    }
}
