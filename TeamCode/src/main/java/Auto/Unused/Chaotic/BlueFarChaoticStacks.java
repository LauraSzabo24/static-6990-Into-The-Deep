package Auto.Unused.Chaotic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
public class BlueFarChaoticStacks extends LinearOpMode {
    OpenCvCamera cam;
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, intakeMotor;
    private Servo clawServo, armLeftServo, armRightServo, lift;
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
        lift = hardwareMap.get(Servo.class, "intakeLiftServo");
        //endregion

        //TRAJECTORIES (left/right in robot perspective)
        Pose2d startPose = new Pose2d(-14, 0, Math.toRadians(90)); //90
        drive.setPoseEstimate(startPose);
        lift.setPosition(0.793); //1 down


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
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                })

                //white pixel
                .lineToLinearHeading(new Pose2d(4,16.5, Math.toRadians(-70)))
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.38);
                    }
                })
                .lineToLinearHeading(new Pose2d(6.2,38.15, Math.toRadians(-60)))
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.345); //0.25
                    }
                })
                .lineToLinearHeading(new Pose2d(-2.2,45.65, Math.toRadians(-60)))
                .lineToLinearHeading(new Pose2d(4.8,43.65, Math.toRadians(-73)))
                .addDisplacementMarker(() -> {
                    double pos = 0.345;
                    for(int i=0; i<4; i++) {
                        lift.setPosition(pos-0.085);
                    }
                })
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.05);
                    }
                })
                .lineToLinearHeading(new Pose2d(0.6,45.65, Math.toRadians(-90)))
                .forward(40)
                .lineToLinearHeading(new Pose2d(-20,5,Math.toRadians(0)))

                //yellow pixel
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

                //white pixel
                .lineToLinearHeading(new Pose2d(-2.4,40.9, Math.toRadians(-73))) //get in range
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.38);
                    }
                })
                .lineToLinearHeading(new Pose2d(7.1,38.8, Math.toRadians(-73))) //removing 8.7 37
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.335); //0.25
                    }
                })
                .lineToLinearHeading(new Pose2d(-2.2,41.65, Math.toRadians(-73))) //back out 40.65
                .lineToLinearHeading(new Pose2d(3,42, Math.toRadians(-73))) //go in 41
                .addDisplacementMarker(() -> {
                    double pos = 0.345;
                    for(int i=0; i<4; i++) {
                        lift.setPosition(pos-0.085);
                    }
                })
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.05);
                    }
                })
                .lineToLinearHeading(new Pose2d(0.6,45.65, Math.toRadians(-90)))
                .forward(40)
                .lineToLinearHeading(new Pose2d(-20,5,Math.toRadians(0)))

                //yellow pixel
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
                //purple pixel
                .lineToLinearHeading(new Pose2d(-5,18.5,Math.toRadians(90)))
               .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .back(7)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(-0.7);
                    }
                })
                .addTemporalMarker(3, () -> {
                    for(int i=0; i<100; i++) {
                        intakeMotor.setPower(0);
                    }
                })
                //white pixel
                .lineToLinearHeading(new Pose2d(4,16.5, Math.toRadians(-70)))
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.38);
                    }
                })
                .lineToLinearHeading(new Pose2d(6.2,37.15, Math.toRadians(-60)))
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.345); //0.25
                    }
                })
                .lineToLinearHeading(new Pose2d(-2.2,45.65, Math.toRadians(-60)))
                .lineToLinearHeading(new Pose2d(4.8,43.65, Math.toRadians(-73)))
                .addDisplacementMarker(() -> {
                    double pos = 0.345;
                    for(int i=0; i<4; i++) {
                        lift.setPosition(pos-0.085);
                    }
                })
                .waitSeconds(0.4)
                .addDisplacementMarker(() -> {
                    for(int i=0; i<100; i++) {
                        lift.setPosition(0.05);
                    }
                })
                .lineToLinearHeading(new Pose2d(0.6,45.65, Math.toRadians(-90)))
                .forward(40)
                .lineToLinearHeading(new Pose2d(-20,5,Math.toRadians(0)))

                //yellow
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

        TrajectorySequence test = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker( () -> {
                    intakeMotor.setPower(-0.7);
                })
                .forward(5)
                .waitSeconds(2)
                .back(1)
                .addDisplacementMarker( () -> {
                    lift.setPosition(0.85);
                })
                .back(1)
                .addDisplacementMarker( () -> {
                    lift.setPosition(0.95);
                })
                .back(4)
                .addDisplacementMarker( () -> {
                    intakeMotor.setPower(0);
                })
                .build();



        waitForStart();
        PropDetectorBLUE.Location place = blueDetector.getLocation();
        telemetry.setMsTransmissionInterval(50);
        if(isStopRequested()) return;

        //DRIVING
        //mailbox
        Mailbox mail =  new Mailbox();
        drive.setPoseEstimate(startPose);
        /*if(place != null) {
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
        }*/
        drive.followTrajectorySequence(test, mail);

    }
}
