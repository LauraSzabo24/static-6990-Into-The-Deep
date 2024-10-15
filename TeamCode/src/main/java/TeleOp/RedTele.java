package TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrives.NewMecanumDrive;

import Autonomous.Mailbox;

@TeleOp
public class RedTele extends LinearOpMode {

    //region PID
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    public static double Kp = 0.01;
    public static double Kd = 0.0;
    public static double targetPosition = 5000;
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
    //endregion

    //region CONTROL STATE
    private enum controlStateB
    {
        FREECONTROL,
        PICKUP,
        LOWDROP,
        HIGHDROP
    }
    private enum controlStateA
    {
        UNLIMITED,
        LIMITED
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

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;

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
        gameModeB = controlStateB.FREECONTROL;
        gameModeA = controlStateA.UNLIMITED;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainLoop();
        }
    }
    public void mainLoop()
    {
        //updates
        drive.update();
        telemetry.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());

        //controls
        switch(gameModeA){
            case UNLIMITED:
                driverAControls();
                break;
            case LIMITED:
                break;
        }


        switch(gameModeB) {

            case FREECONTROL:
                //cool
                break;
            case PICKUP:
                //cooler
                break;
            case LOWDROP:
                //super cool
                break;
            case HIGHDROP:
                //extra cool
                break;
        }
    }

    public void driverAControls()
    {
        //drivetrain changes in speed | right trigger slow | left trigger fast
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

    }

}
