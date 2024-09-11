//package Ancient;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import Auto.Mailbox;
//import Camera.PropDetectorRED;
//
//@Autonomous
//@Disabled
//public class OldBlue extends LinearOpMode {
//    OpenCvCamera cam;
//    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, intakeMotor;
//    private Servo clawServo, armLeftServo, armRightServo, intakeLift;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        int cameraMonitorViewId = hardwareMap.appContext
//                .getResources().getIdentifier("cameraMonitorViewId",
//                        "id", hardwareMap.appContext.getPackageName());
//
//        WebcamName camera = hardwareMap.get(WebcamName.class, "camera");
//        OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);
//
//        PropDetectorRED redDetector = new PropDetectorRED(telemetry);
//        cam.setPipeline(redDetector);
//        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                telemetry.addLine("CAMERA WORKS");
//                telemetry.update();
//                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//                telemetry.addData("THE CAMERA DID NOT OPEN PROPERLY SEND HELP", errorCode);
//                telemetry.update();
//            }
//        });
//
//        sleep(20);
//
//        //MOTORS AND SERVOS
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
//        clawServo = hardwareMap.get(Servo.class, "claw");
//        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
//        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
//        intakeLift = hardwareMap.get(Servo.class, "intakeLiftServo");
//
//
//        //TRAJECTORIES (left/right in robot perspective)
//        Pose2d startPose = new Pose2d(-14, 0, Math.toRadians(90)); //90
//        drive.setPoseEstimate(startPose);
//
//
//        //purple pixel
//       /* TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-6,-19,Math.toRadians(-90)))
//                //.lineToLinearHeading(new Pose2d(-22,-37,Math.toRadians(0)))
//                //purple pixel
//                .addTemporalMarker(0.8, () -> {
//                    intakeLift.setPosition(0.75);
//                    intakeMotor.setPower(-0.7);
//                })
//                .addTemporalMarker(2, () -> {
//                    intakeMotor.setPower(0);
//                    intakeLift.setPosition(0.25);
//                })
//
//                .lineToLinearHeading(new Pose2d(0,-53, Math.toRadians(0)))
//                .addDisplacementMarker(() -> {
//                    //mailbox
//                    Mailbox mail =  new Mailbox();
//                    mail.setAutoEnd(Math.toDegrees(drive.getExternalHeading()));
//                })
//                .build();
//        TrajectorySequence centerPurple = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-6,-19,Math.toRadians(-90)))
//
//                //purple pixel
//                .addDisplacementMarker(() -> {
//                    intakeLift.setPosition(0.75);
//                    for(int i=0; i<100; i++) {
//                        intakeMotor.setPower(-0.7);
//                    }
//                })
//                .addTemporalMarker(3, () -> {
//                    intakeMotor.setPower(0);
//                    intakeLift.setPosition(0.25);
//                })
//
//                //white pixel
//                .strafeRight(19)
//                .lineToLinearHeading(new Pose2d(0,-53, Math.toRadians(0)))
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        intakeMotor.setPower(0.7);
//                    }
//                })
//                .forward(5)
//                .back(10)
//                .addDisplacementMarker(() -> {
//                    intakeLift.setPosition(0.75);
//                    for(int i=0; i<100; i++) {
//                        intakeMotor.setPower(0);
//                    }
//                    intakeLift.setPosition(0.25);
//                })
//                .build();*/
//
//        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-6,16.5,Math.toRadians(90)))
//                //.lineToLinearHeading(new Pose2d(-6,-16,Math.toRadians(90)))
//                //purple pixel
//                .addDisplacementMarker(() -> {
//                    for(int i=0; i<100; i++) {
//                        intakeLift.setPosition(0.75);
//                        intakeMotor.setPower(-0.7);
//                        /*armLeftServo.setPosition(0);
//                        armRightServo.setPosition(1);*/
//                    }
//                })
//                .addTemporalMarker(3, () -> {
//                    for(int i=0; i<100; i++) {
//                        /*clawServo.setPosition(0.8);
//                        armLeftServo.setPosition(1);
//                        armRightServo.setPosition(0);*/
//                        intakeMotor.setPower(0);
//                        intakeLift.setPosition(0.25);
//                    }
//                })
//
//                //white pixel
//                .strafeLeft(19)
//                .lineToLinearHeading(new Pose2d(0,53, Math.toRadians(0)))
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        intakeMotor.setPower(0.4);
//                    }
//                })
//                .forward(7)
//                .waitSeconds(2)
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        intakeMotor.setPower(0.7);
//                    }
//                })
//                .back(5)
//                .forward(5)
//                .addDisplacementMarker(() -> {
//                    for(int i=0; i<100; i++) {
//                        intakeLift.setPosition(0.75);
//                    }
//                    for(int i=0; i<100; i++) {
//                        intakeLift.setPosition(0.25);
//                        clawServo.setPosition(0.45);
//                    }
//                })
//                .back(115)
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        intakeMotor.setPower(0);
//                    }
//                })
//                .strafeRight(35)
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        armLeftServo.setPosition(0.2);
//                        armRightServo.setPosition(0.8);
//                    }
//                })
//                .back(10)
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        clawServo.setPosition(0.8);
//                    }
//                })
//                .forward(5)
//                .strafeLeft(6)
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        armLeftServo.setPosition(1);
//                        armRightServo.setPosition(0);
//                        clawServo.setPosition(0.45);
//                    }
//                })
//                .waitSeconds(1)
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        armLeftServo.setPosition(0.2);
//                        armRightServo.setPosition(0.8);
//                    }
//                })
//                .waitSeconds(0.5)
//                .addDisplacementMarker( () -> {
//                    for(int i=0; i<100; i++) {
//                        clawServo.setPosition(0.8);
//                    }
//                })
//                .back(5)
//
//                .build();
//
//        /*TrajectorySequence main = drive.trajectorySequenceBuilder(leftPurple.end())
//                //yellow pixel
//                .addDisplacementMarker(() -> {
//                    //mailbox
//                    Mailbox mail =  new Mailbox();
//                    mail.setAutoEnd(Math.toDegrees(drive.getExternalHeading()));
//                })
//
//
//                .build();*/
//
//
//
//        waitForStart();
//        PropDetectorRED.Location place = redDetector.getLocation();
//        armLeftServo.setPosition(1);
//        armRightServo.setPosition(0);
//        clawServo.setPosition(0.5);
//
//        telemetry.setMsTransmissionInterval(50);
//        if(isStopRequested()) return;
//
//        //DRIVING
//        drive.setPoseEstimate(startPose);
//        /*if(place != null) {
//            switch (place) {
//                case NOT_FOUND:
//                    drive.followTrajectorySequence(rightPurple);
//                    break;
//                case CENTER:
//                    drive.followTrajectorySequence(centerPurple);
//                    break;
//                case LEFT:
//                    drive.followTrajectorySequence(leftPurple);
//
//            }
//        }
//        else{
//            drive.followTrajectorySequence(rightPurple);
//        }*/
//
//        drive.followTrajectorySequence(leftPurple);
//        //drive.followTrajectorySequence(main);
//
//        //mailbox
//        Mailbox mail =  new Mailbox();
//        mail.setAutoEnd(Math.toDegrees(drive.getExternalHeading()));
//
//    }
//}

