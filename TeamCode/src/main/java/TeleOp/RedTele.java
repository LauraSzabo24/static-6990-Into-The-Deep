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
    public static double flpP = 0.004, flpI = 0.001, flpD = 0.00014, flpF = 0.001;
    public static int flpTarget;
    private PIDController ext;
    public static double extP = 0.004, extI = 0.00, extD = 0.00015, extF = 0.003;
    public static int extTarget;
    public static int divider = 1;
    FtcDashboard dashboard;
    //endregion

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
    boolean pickupTwo = false;
    ElapsedTime jerkTimer = new ElapsedTime();
    boolean jerked = false;
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

    //INITIALIZATIONS
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
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void movementInit()
    {
        wristRServo.setPosition(0);
        wristLServo.setPosition(0.8);

        clawLServo.setPosition(0.2);
        clawRServo.setPosition(0.6);

        spinnerServo.setPosition(0.1522);
        flpTarget = -200;
        extTarget = 0;
    }

    //BASICS
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();
        movementInit();
        clawIH = true;
        gameModeB = controlStateB.FREE;
        gameModeA = controlStateA.UNLIMITED;

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
        //telemetry.addData("GAMEMODE A", gameModeA);
        stateCheck();

        //EXTENDER & FLIPPER
        extCONTROLLER();
        flpCONTROLLER();
    }

    //DRIVING CONTROLS
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
    public void driverBControls()
    {
        setStates();
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
            wristRServo.setPosition(0);
            wristLServo.setPosition(0.8);
            gameModeB = controlStateB.FREE;
        }

        //SPINNER
        if(gamepad2.right_stick_x>0 || gamepad2.right_stick_x<0)
        {
            telemetry.addLine("SPINNER MOVEMENT");
            spinnerServo.setPosition(spinnerServo.getPosition() + (gamepad2.right_stick_x * 0.05));
            gameModeB = controlStateB.FREE;
        }
        if(gamepad2.right_stick_button)
        {
            spinnerServo.setPosition(0.1522);
        }

        //EXTENDER
        if(gamepad2.dpad_up && extTarget<=1500)
        {
            telemetry.addLine("ext UP");
            gameModeB = controlStateB.FREE;
            if(extTarget+50>=1340)
            {
                extTarget+=Math.abs(Math.abs(extTarget)-1500);
            }
            else {
                extTarget+=50;
            }
        }
        else if(gamepad2.dpad_down && extTarget>=0)
        {
            telemetry.addLine("ext DOWN");
            gameModeB = controlStateB.FREE;
            if(extTarget-50<=0)
            {
                extTarget-=Math.abs(extTarget);
            }
            else {
                extTarget-=50;
            }
        }

        //FLIPPER
        if(gamepad2.dpad_left && flpTarget<=-150)
        {
            telemetry.addLine("flp DOWN");
            gameModeB = controlStateB.FREE;

            if(flpTarget+40>=-150)
            {
                flpTarget+=Math.abs(flpTarget+150);
            }
            else {
                flpTarget+=40;
            }
        }
        else if(gamepad2.dpad_right && flpTarget>=-1650)
        {
            telemetry.addLine("flp UP");
            gameModeB = controlStateB.FREE;

            if(flpTarget-40<=-1650)
            {
                flpTarget-=Math.abs(flpTarget+1650);
            }
            else {
                flpTarget-=40;
            }
        }
    }
    public void setStates()
    {
        if(gameModeB!= controlStateB.HIGH && currG2.y && !oldG2.y)
        {
            divider = 4;
            telemetry.addLine("TO HIGH POSITION");
            if(flpTarget>-700) {
                jerkTimer.reset();
                while(jerkTimer.time() < 0.2) {
                    flpTarget = -740;
                    flpCONTROLLER();
                }
            }
            else {
                gameModeB = controlStateB.HIGH;
                jerkTimer.reset();
                while (jerkTimer.time() < 1.0) {
                    extTarget = 0;
                    flpTarget = -740;
                    flpCONTROLLER();
                    extCONTROLLER();
                }

                jerkTimer.reset();
                while (jerkTimer.time() < 0.5) {
                    extTarget = 1500;
                    spinnerServo.setPosition(0.6989);
                    wristRServo.setPosition(0.555);
                    wristLServo.setPosition(0.2444);
                    extCONTROLLER();
                }
            }
        }
        else if(gameModeB!= controlStateB.LOW && currG2.x && !oldG2.x)
        {
            divider = 4;
            telemetry.addLine("TO LOW POSITION");
            gameModeB = controlStateB.LOW;

            jerkTimer.reset();
            while (jerkTimer.time() < 0.5) {
                extTarget = 0;
                flpTarget = -250;
                wristRServo.setPosition(0);
                wristLServo.setPosition(0.8);
                spinnerServo.setPosition(0.1522);
                flpCONTROLLER();
                extCONTROLLER();
            }
        }
        else if(gameModeB!= controlStateB.PICKUP && currG2.a && !oldG2.a)
        {
            divider = 4;
            jerkTimer.reset();
            while(jerkTimer.time() < 1.0) {
                extTarget = 0;
                extCONTROLLER();
            }
            clawLServo.setPosition(0.6);
            clawRServo.setPosition(0.2);

            if(!pickupTwo) {
                telemetry.addLine("PICKUP LOW");
                wristRServo.setPosition(0.2);
                wristLServo.setPosition(0.8);
                pickupTwo = true;
            }
            else if( gameModeB != controlStateB.PICKUP) {
                telemetry.addLine("PICKUP HANG");
                wristRServo.setPosition(0.0556);
                wristLServo.setPosition(0.822);
                pickupTwo = false;
            }
            gameModeB = controlStateB.PICKUP;
            spinnerServo.setPosition(0.1522);
            flpTarget = -1500;
            clawLServo.setPosition(0.6);
            clawRServo.setPosition(0.2);
        }

        //JERK
        if(currG2.right_bumper && !oldG2.right_bumper)
        {
            divider = 4;
            jerkTimer.reset();
            wristRServo.setPosition(0.8);
            wristLServo.setPosition(0.2);
            flpTarget = -600;

            jerkTimer.reset();
            while(jerkTimer.time() < 1.0)
            {
                flpCONTROLLER();
            }
            wristRServo.setPosition(0.555);
            wristLServo.setPosition(0.2444);
            clawLServo.setPosition(0.6);
            clawRServo.setPosition(0.2);
        }
        divider = 1;
    }

    //CONTROLLERS
    public void extCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = armMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget) + (Math.cos(Math.toRadians(extTarget/ticksPerDegree)) * extF);
        armMotor.setPower(extPwr *(1/3.0));

        telemetry.addData("extPos ", extPose);
        telemetry.addData("extTarget ", extTarget);
    }
    public void flpCONTROLLER()
    {
        flp.setPID(flpP, flpI, flpD);
        int flpPose = flipMotor.getCurrentPosition();
        double flpPwr = flp.calculate(flpPose, flpTarget) + Math.cos(Math.toRadians(flpTarget/ticksPerDegree)) * flpF;
        flipMotor.setPower(flpPwr * (1/divider));

        telemetry.addData("flpPos ", flpPose);
        telemetry.addData("flpTarget ", flpTarget);
    }

    //TELEMETRY
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
        telemetry.addData("spinner - ", spinnerServo.getPosition());
        telemetry.addData("wrist L - ", wristLServo.getPosition());
        telemetry.addData("wrist R - ", wristRServo.getPosition());
    }
}
