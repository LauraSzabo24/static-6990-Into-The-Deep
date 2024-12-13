package Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Unused.DI_MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import TeleOp.RedTele;

@Autonomous
public class Red4Specimen extends OpMode {
    //region FLIPPER CONTROLLER
    //POSITION
    ElapsedTime timer = new ElapsedTime();
    private double flpPosError = 0;
    private double flpPosISum = 0;

    public static double flpPP = 10, flpPI = 0, flpPD = 0;
    public static int flpPosTarget = 0;

    //VELOCITY
    private double flpVeloError = 0;
    private double flpVeloISum = 0;
    public static int flpVeloTarget = 0;
    public static int testTarget = 1200;
    public static double flpVP = 0.0002, flpVI = 0.004, flpVD = 0.0000001;  //RISING
    //endregion

    //region EXTENDER CONTROLLER
    public static double ticksPerDegree = 537.7;
    private PIDController ext;
    public static double extP = 0.004, extI = 0.00, extD = 0.00015, extF = 0.003;
    public static int extTarget;
    FtcDashboard dashboard;
    //endregion

    //region DRIVER A MATERIAL
    IMU imu;
    IMU.Parameters parameters;
    Pose2d startPose;
    NewMecanumDrive drive;
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
    //endregion

    //region DRIVER B MATERIAL
    private Servo wristServo, spinnerServo, clawServo;
    DcMotorEx flipMotor, armMotor;
    boolean clawIH;
    boolean pickupTwo = false;
    ElapsedTime jerkTimer = new ElapsedTime();
    boolean jerked = false;
    double extLimit = 1600;
    double flpLimit = 4300;

    int spinnerPos = 0;
    //endregion
    @Override
    public void init()
    {
        telemetry.setMsTransmissionInterval(50);
        Mailbox mail = new Mailbox();
        hardwareInit();
        movementInitII();
        startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //region PRELOAD DROP OFF
        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-10, 29), Math.toRadians(90))
                .addDisplacementMarker(1,() -> {
                    extTarget = 1600;
                    flpPosTarget = -1500;
                })
                .addDisplacementMarker(20,() -> {
                    spinnerServo.setPosition(0.77);
                    wristServo.setPosition(0.95);//0.8389
                })
                .addTemporalMarker(3,() -> {
                    spinnerServo.setPosition(0.77);
                    wristServo.setPosition(1);
                    extTarget = 1140;
                })
                .addTemporalMarker(4,() -> {
                    clawServo.setPosition(0);
                })
                .waitSeconds(2)
                .forward(3)
                //.splineToConstantHeading(new Vector2d(-20, 33), Math.toRadians(90))

                .build();
        //endregion

        //region CONNECTION
        TrajectorySequence connection = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-36, 30), Math.toRadians(-90))
                .forward(10)
                .build();
        //endregion

        //region INTO TERMINAL
        TrajectorySequence terminalCycles = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-47, 10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-47, 55), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(-58, 10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-58, 55), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(-65, 10), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-65, 55), Math.toRadians(-90))

                .splineToConstantHeading(new Vector2d(-55, 35), Math.toRadians(-90))
                .build();
        //endregion

        //region CYCLE ONE
        TrajectorySequence cycleOne = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .forward(10)
                .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-90))
                .waitSeconds(2)
                .lineTo(new Vector2d(-55, 40))
                .back(10)
                .forward(10)
                .build();
        //endregion

        //region CYCLE TWO
        TrajectorySequence cycleTwo = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-90))
                .waitSeconds(2)
                .lineTo(new Vector2d(-55, 40))
                .back(10)
                .forward(10)
                .build();
        //endregion

        //region CYCLE THREE
        TrajectorySequence cycleThree = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-90))
                .forward(3)
                .waitSeconds(2)
                .build();
        //endregion

        //DRIVING
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequenceAsync(preload, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(-180))));
    }
    @Override
    public void loop()  {
        drive.update();
        extCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }
    public void highPosition()
    {
        telemetry.addLine("TO HIGH POSITION");
        if(flpPosTarget>-2000) {
            extTarget = 1600;
            flpPosTarget = -1650;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
            }
            spinnerServo.setPosition(0.77);
            wristServo.setPosition(0.8389);
        }
        else {
            /*jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                extTarget = 0;
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            armMotor.setPower(0);

            spinnerServo.setPosition(0.77);
            wristServo.setPosition(0.8389);
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                flpPosTarget = -1650;
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                extTarget = 1600;
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());

            }*/
        }
    }
    public void lowPosition()
    {
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 0;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        armMotor.setPower(0);

        spinnerServo.setPosition(0.77);
        wristServo.setPosition(0.7989);
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            flpPosTarget = -1160;
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 440;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());

        }
    }
    public void homePosition()
    {
        telemetry.addLine("TO HOME POSITION");
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 250;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }

        wristServo.setPosition(0.39);
        spinnerServo.setPosition(0.21);
        clawServo.setPosition(0.4);
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            flpPosTarget = -200;
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
    }
    public void pickupPosition()
    {
        telemetry.addLine("TO PICKUP POSITION");
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 250;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        armMotor.setPower(0);
        spinnerServo.setPosition(0.215);
        wristServo.setPosition(0.595);
        clawServo.setPosition(0);
        jerkTimer.reset();
        while (jerkTimer.time() < 0.5) {
            flpPosTarget = -3671;
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
    }
    public void jerk()
    {
        while (jerkTimer.time() < 3) {
        }
        spinnerServo.setPosition(0.77);
        wristServo.setPosition(0.95);
        extTarget = 1140;
        jerkTimer.reset();
        while (jerkTimer.time() < 0.6) {
        }
        clawServo.setPosition(0);
    }
    public void extCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = armMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget) + (Math.cos(Math.toRadians(extTarget/ticksPerDegree)) * extF);
        armMotor.setPower(extPwr *(1/3.0));

        telemetry.addData("extPos ", extPose);
        telemetry.addData("extTarget ", extTarget);
    }
    public void flpCONTROLLER(int target, int state) //in with the target -> out with the velocity
    {
        int currError = target - state;
        double time = timer.seconds();
        timer.reset();
        flpPosISum += currError * time;
        double deriv = (currError - flpPosError)/time;
        flpPosError = currError;

        double velocityVal = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        flipMotor.setVelocity(velocityVal);

        /*double veloTarget = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        telemetry.addData("targetvelo",veloTarget);
        double neededPower = flpVelocityCONTROLLER(veloTarget, flipMotor.getVelocity(), time);
        telemetry.addData("power",neededPower);
        flipMotor.setPower(neededPower);*/
    }
    public void hardwareInit()
    {
        //RANDOM
        dashboard = FtcDashboard.getInstance();

        //PID
        ext = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motors
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        wristServo = hardwareMap.get(Servo.class, "wrist");
        spinnerServo = hardwareMap.get(Servo.class, "spinner");
        clawServo = hardwareMap.get(Servo.class, "claw");

        //mailbox
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void movementInitII()
    {
        wristServo.setPosition(0.39);
        clawServo.setPosition(0.4);
        spinnerServo.setPosition(0.21);
        flpPosTarget = -200;
        extTarget = 0;
    }

}