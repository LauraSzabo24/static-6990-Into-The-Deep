package TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrives.NewMecanumDrive;

import Autonomous.Mailbox;

@TeleOp
@Config
public class RedTele extends LinearOpMode {

    //region EXTENDER FLIPPER CONTROLS
    public static double ticksPerDegree = 537.7;
    private PIDController flp;
    public static double flpP = 0.004, flpI = 0.0005, flpD = 0.00014, flpF = 0.001;
    public static int flpTarget = -100;
    private PIDController ext;
    public static double extP = 0.001, extI = 0.00, extD = 0.0, extF = 0;
    public static int extTarget = -100;
    FtcDashboard dashboard;

    //region DRIVER A MATERIAL
    IMU imu;
    IMU.Parameters parameters;
    NewMecanumDrive drive;
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
    //endregion

    //region DRIVER B MATERIAL
    private Servo wristLServo, wristRServo, spinnerServo, clawLServo, clawRServo;
    DcMotorEx flipMotor, armMotor;
    boolean clawIH;
    ElapsedTime jerkTimer = new ElapsedTime();
    //endregion

    //region GAMEPADS
    Gamepad currG1;
    Gamepad oldG1;
    Gamepad currG2;
    Gamepad oldG2;

    //endregion

    //region CONTROL STATE
    private enum controlStateB
    {
        FREE,
        PICKUP,
        LOW,
        HIGH
    }
    private enum controlStateA
    {
        UNLIMITED,
        LIMITED,
        HANG
    }
    controlStateB gameModeB;
    controlStateA gameModeA;
    //endregion

    public void hardwareInit()
    {
        //RANDOM
        dashboard = FtcDashboard.getInstance();

        //PID
        flp = new PIDController(flpP, flpI, flpD);
        ext = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //drive motors
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        wristLServo = hardwareMap.get(Servo.class, "wristL");
        wristRServo = hardwareMap.get(Servo.class, "wristR");
        spinnerServo = hardwareMap.get(Servo.class, "spinner");
        clawLServo = hardwareMap.get(Servo.class, "clawL");
        clawRServo = hardwareMap.get(Servo.class, "clawR");

        //gamepads
        currG1 = new Gamepad();
        oldG1 = new Gamepad();
        currG2 = new Gamepad();
        oldG2 = new Gamepad();


        //mailbox
        Mailbox mail =  new Mailbox();
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();
        clawIH = true;
        gameModeB = controlStateB.FREE;
        gameModeA = controlStateA.UNLIMITED;
        movementInit();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainLoop();
        }
    }
    public void mainLoop()
    {
        //UPDATES
        oldG1.copy(currG1);
        oldG2.copy(currG2);
        currG1.copy(gamepad1);
        currG2.copy(gamepad2);
        telemetry.update();
        drive.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());

        //CONTROLS
        switch(gameModeA){
            case UNLIMITED:
                driverAControls();
                break;
            case LIMITED:
                break;
        }
        driverBControls();

        telemetry.addData("GAMEMODE B", gameModeB);
        telemetry.addData("GAMEMODE A", gameModeA);
        stateCheck();

        //EXTENDER & FLIPPER
        extCONTROLLER();
        flpCONTROLLER();
    }

    public void extCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = armMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget) + (Math.cos(Math.toRadians(extTarget/ticksPerDegree)) * extF);
        armMotor.setPower(extPwr);

        telemetry.addData("extPos ", extPose);
        telemetry.addData("extTarget ", extTarget);
    }

    public void flpCONTROLLER()
    {
        flp.setPID(flpP, flpI, flpD);
        int flpPose = flipMotor.getCurrentPosition();
        double flpPwr = flp.calculate(flpPose, flpTarget) + Math.cos(Math.toRadians(flpTarget/ticksPerDegree)) * flpF;
        flipMotor.setPower(flpPwr);

        telemetry.addData("flpPos ", flpPose);
        telemetry.addData("flpTarget ", flpTarget);
    }
    public void driverAControls()
    {
        //SPEED CHANGES | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
        multiply = 1;
        speed = 3;
        if (gamepad1.right_trigger > 0) {
            multiply = 0.5;
            speed = 3;
        }
        if (gamepad1.left_trigger > 0) {
            multiply =0.7;
            speed = 1;
        }

        //FIELD CENTRIC
        poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -((gamepad1.left_stick_y)* multiply)/speed,
                -((gamepad1.left_stick_x)* multiply)/speed
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        ((gamepad1.right_stick_x * multiply)/speed)
                )
        );
        drive.update();
    }

    public void stateCheck()
    {
        if(clawIH)
        {
            telemetry.addLine("CLAW CLOSE");
        }
        else{
            telemetry.addLine("CLAW OPEN");
        }

        telemetry.addData("flp TARGET - ", flpTarget);
        telemetry.addData("ext TARGET - ", extTarget);
    }

    public void movementInit()
    {
        wristRServo.setPosition(1);
        wristLServo.setPosition(1);
        clawLServo.setPosition(0.6);
        clawRServo.setPosition(0.2);
    }
    public void driverBControls()
    {
        //PRESET POSES
        if(gameModeB!= controlStateB.HIGH && currG2.y && !oldG2.y)
        {
            telemetry.addLine("TO HIGH POSITION");
            gameModeB = controlStateB.HIGH;

            /*wristLServo.setPosition(0);
            wristRServo.setPosition(0);
            spinnerServo.setPosition(0);
            flpTarget = 0;
            extTarget = 0;*/
        }
        else if(gameModeB!= controlStateB.LOW && currG2.x && !oldG2.x)
        {
            telemetry.addLine("TO LOW POSITION");
            gameModeB = controlStateB.LOW;

            /*wristLServo.setPosition(0);
            wristRServo.setPosition(0);
            spinnerServo.setPosition(0);
            flpTarget = 0;
            extTarget = 0;*/
        }
        else if(gameModeB!= controlStateB.PICKUP && currG2.a && !oldG2.a)
        {
            telemetry.addLine("TO PICKUP POSITION");
            gameModeB = controlStateB.PICKUP;

            /*wristLServo.setPosition(0);
            wristRServo.setPosition(0);
            spinnerServo.setPosition(0);
            flpTarget = 0;
            extTarget = 0;*/
        }

        //CLAW
        if (currG2.b && !oldG2.b)
        {
            if(clawIH)
            {
                clawLServo.setPosition(0.6);
                clawRServo.setPosition(0.2);
            }
            else {
                clawLServo.setPosition(0.2);
                clawRServo.setPosition(0.6);
            }
            clawIH = !clawIH;
        }

        //JERK
        if(currG2.right_bumper && !oldG2.right_bumper)
        {
            telemetry.addLine("JERK");
            /*wristLServo.setPosition(wristLServo.getPosition()+0.1);
            wristRServo.setPosition(wristRServo.getPosition()+0.1);
            extTarget = extTarget-10;
            flpTarget = flpTarget-10;
            //timer goes here wait 1 sec
            wristLServo.setPosition(wristLServo.getPosition()-0.1);
            wristRServo.setPosition(wristRServo.getPosition()-0.1);
            extTarget = extTarget+10;
            flpTarget = flpTarget+10;*/
        }

        //WRIST
        if(gamepad2.left_stick_x>0 || gamepad2.left_stick_x<0)
        {
            telemetry.addLine("WRIST MOVEMENT");
            wristRServo.setPosition(wristRServo.getPosition() - (currG2.left_stick_x * 0.05));
            wristLServo.setPosition(wristLServo.getPosition() + (currG2.left_stick_x * 0.05));
            gameModeB = controlStateB.FREE;
        }
        if(gamepad2.left_stick_button)
        {
            telemetry.addLine("WRIST MOVEMENT");
            wristRServo.setPosition(1);
            wristLServo.setPosition(1);
            gameModeB = controlStateB.FREE;
        }

        //SPINNER
        if(currG2.right_stick_x>0 || currG2.right_stick_x<0)
        {
            telemetry.addLine("SPINNER MOVEMENT");
            spinnerServo.setPosition(spinnerServo.getPosition() + (currG2.right_stick_x * 0.05));
            gameModeB = controlStateB.FREE;
        }

        //EXTENDER
        if(gamepad2.dpad_up && extTarget<=1600)
        {
            telemetry.addLine("ext UP");
            gameModeB = controlStateB.FREE;
            if(extTarget+50>=1420)
            {
                extTarget+=Math.abs(Math.abs(extTarget)-1600);
            }
            else {
                extTarget+=50;
            }
        }
        else if(gamepad2.dpad_down && extTarget>=-20)
        {
            telemetry.addLine("ext DOWN");
            gameModeB = controlStateB.FREE;
            if(extTarget-50<=-20)
            {
                extTarget-=Math.abs(Math.abs(extTarget)-20);
            }
            else {
                extTarget-=50;
            }
        }

        //FLIPPER
        else if(gamepad2.dpad_left && flpTarget<=-100)
        {
            telemetry.addLine("flp DOWN");
            gameModeB = controlStateB.FREE;

            if(flpTarget+80>=-100)
            {
                flpTarget+=Math.abs(flpTarget+100);
            }
            else {
                flpTarget+=80;
            }
        }
        else if(gamepad2.dpad_right && flpTarget>=-1580)
        {
            telemetry.addLine("flp UP");
            gameModeB = controlStateB.FREE;

            if(flpTarget-80<=-1580)
            {
                flpTarget-=Math.abs(flpTarget+1580);
            }
            else {
                flpTarget-=80;
            }
        }

        //PULSES
        if(currG2.dpad_right && !oldG2.dpad_right && flpTarget>=-1580)
    {
        telemetry.addLine("flp UP");
        gameModeB = controlStateB.FREE;

        if(flpTarget-10<=-1580)
        {
            flpTarget-=Math.abs(flpTarget+1580);
        }
        else {
            flpTarget-=10;
        }
    }
        else if (currG2.dpad_left && !oldG2.dpad_left && flpTarget<=-100) {
            telemetry.addLine("flp DOWN");
            gameModeB = controlStateB.FREE;

            if (flpTarget + 10 >= -100) {
                flpTarget += Math.abs(flpTarget + 100);
            } else {
                flpTarget += 10;
            }
        }
    }

}
