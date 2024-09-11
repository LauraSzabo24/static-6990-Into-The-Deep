package Auto.BlueFar;

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

import Auto.Mailbox;
import Camera.PropDetectorBLUE;

@Autonomous
public class BlueFarParkCenterDD extends LinearOpMode {
    OpenCvCamera cam;
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, intakeMotor;
    private Servo clawServo, armLeftServo, armRightServo, intakeLift;
    @Override
    public void runOpMode() throws InterruptedException {
        //region CAMERA
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
        //endregion

        //region MOTORS AND SERVOS
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
        clawServo = hardwareMap.get(Servo.class, "claw");
        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
        intakeLift = hardwareMap.get(Servo.class, "intakeLiftServo");
        //endregion

        //TRAJECTORIES (left/right in robot perspective)
        Pose2d startPose = new Pose2d(-14, 0, Math.toRadians(90)); //90
        drive.setPoseEstimate(startPose);


        //region RIGHT
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-10,22,Math.toRadians(120)))
                .strafeLeft(15)
                .forward(7)
                //purple pixel
               .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })

                .waitSeconds(1)
                .back(16)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                })
                //yellow pixel
                .lineToLinearHeading(new Pose2d(-5,53, Math.toRadians(0)))
                .waitSeconds(1)
                .back(95)
                .addDisplacementMarker( () -> {
                    for(int i=0; i<100; i++) {
                        clawServo.setPosition(0);
                    }
                })
                .strafeRight(27) //31

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
        //endregion

        //region CENTER
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-3,40,Math.toRadians(180)))
                .waitSeconds(1)
                //purple pixel
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .back(4)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                })

                //yellow pixel
                .lineToLinearHeading(new Pose2d(-5,53, Math.toRadians(0)))
                .waitSeconds(1)
                .back(95) //100
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
        //endregion

        //region LEFT
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-5,16.5,Math.toRadians(90)))
                //purple pixel
               .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .back(5)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .forward(8)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .waitSeconds(1)

                //yellow pixel
                .strafeLeft(19)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                })
                .lineToLinearHeading(new Pose2d(-5,53, Math.toRadians(0)))
                .waitSeconds(1)
                .back(95)
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
        //endregion

        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose) //(-14, 0_)
                .turn(Math.toRadians(52))
                .build();

        waitForStart();
        PropDetectorBLUE.Location place = blueDetector.getLocation();
        telemetry.setMsTransmissionInterval(50);
        if(isStopRequested()) return;

        //DRIVING
        //mailbox
        Mailbox mail =  new Mailbox();
        drive.setPoseEstimate(startPose);
       if(place != null) {
            switch (place) {
                case NOT_FOUND:
                    drive.followTrajectorySequence(right, mail);
                    break;
                case CENTER:
                    drive.followTrajectorySequence(center, mail);
                    break;
                case LEFT:
                    drive.followTrajectorySequence(left, mail);

            }
        }
        else{
            drive.followTrajectorySequence(right, mail);
        }

       // drive.followTrajectorySequence(test, mail);
        mail.setAutoEnd(drive.getPoseEstimate());

    }
}
