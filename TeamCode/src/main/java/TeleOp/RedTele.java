package TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class RedTele extends LinearOpMode {

    //region EXTENDER PID
    ElapsedTime timer = new ElapsedTime();
    private double extError = 0;
    private double extISum = 0;
    public static double extP = 0.01;
    public static double extD = 0.0;
    public static double extTarget = 5000;
    //endregion

    //region FLIPPER PID
    private double flpError = 0;
    private double flpISum = 0;
    public static double flpP = 0.01;
    public static double flpD = 0.0;
    public static double flpTarget = 5000;
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
        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flpTarget = 0;

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extTarget = 0;

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

        //EXTENDER & FLIPPER
        extPID();
        flpPID();
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
    public void driverBControls()
    {
        //AUTO POSES
        if(gameModeB!= controlStateB.HIGH && currG2.y && !oldG2.y)
        {
            telemetry.addLine("TO HIGH POSITION");
            gameModeB = controlStateB.HIGH;

            wristLServo.setPosition(0);
            wristRServo.setPosition(0);
            spinnerServo.setPosition(0);
            flpTarget = 0;
            extTarget = 0;
        }
        else if(gameModeB!= controlStateB.LOW && currG2.x && !oldG2.x)
        {
            telemetry.addLine("TO LOW POSITION");
            gameModeB = controlStateB.LOW;

            wristLServo.setPosition(0);
            wristRServo.setPosition(0);
            spinnerServo.setPosition(0);
            flpTarget = 0;
            extTarget = 0;
        }
        else if(gameModeB!= controlStateB.PICKUP && currG2.a && !oldG2.a)
        {
            telemetry.addLine("TO PICKUP POSITION");
            gameModeB = controlStateB.PICKUP;

            wristLServo.setPosition(0);
            wristRServo.setPosition(0);
            spinnerServo.setPosition(0);
            flpTarget = 0;
            extTarget = 0;
        }

        //CLAW
        if (currG2.b && !oldG2.b)
        {
            telemetry.addLine("CLAW TOGGLE");
            if(clawIH)
            {
                clawLServo.setPosition(0.5);
                clawRServo.setPosition(0.5);
            }
            else {
                clawLServo.setPosition(0);
                clawRServo.setPosition(0);
            }
            clawIH = !clawIH;
        }

        //JERK
        if(currG2.right_bumper && !oldG2.right_bumper)
        {
            telemetry.addLine("JERK");
            wristLServo.setPosition(wristLServo.getPosition()+0.1);
            wristRServo.setPosition(wristRServo.getPosition()+0.1);
            extTarget = extTarget-10;
            flpTarget = flpTarget-10;
            //timer goes here wait 1 sec
            wristLServo.setPosition(wristLServo.getPosition()-0.1);
            wristRServo.setPosition(wristRServo.getPosition()-0.1);
            extTarget = extTarget+10;
            flpTarget = flpTarget+10;
        }

        //WRIST
        if(currG2.left_stick_y>0 || currG2.left_stick_y<0)
        {
            wristRServo.setPosition(wristRServo.getPosition() + (currG2.left_stick_y * 0.5));
            wristLServo.setPosition(wristRServo.getPosition() + (currG2.left_stick_y * 0.5));
            gameModeB = controlStateB.FREE;
        }

        //SPINNER
        if(currG2.right_stick_x>0 || currG2.right_stick_x<0)
        {
            spinnerServo.setPosition(spinnerServo.getPosition() + (currG2.right_stick_x * 0.5));
            gameModeB = controlStateB.FREE;
        }

        //EXTENDER & FLIPPER
        if(currG2.dpad_up && extTarget<(5000-100))
        {
            extTarget+=100;
        }
        else if(currG2.dpad_down && extTarget>(0+100))
        {
            extTarget-=100;
        }
        else if(currG2.dpad_left && flpTarget<(5000-100))
        {
            flpTarget+=100;
        }
        else if(currG2.dpad_right && flpTarget>(0+100))
        {
            flpTarget-=100;
        }
    }

    public void extPID()
    {
        double pidval = (extP * timer.seconds()) + (extISum/timer.seconds()) + (extD*(extError-);

        timer.reset();
    }
    public void flpPID() //bang bang?
    {

    }

}
