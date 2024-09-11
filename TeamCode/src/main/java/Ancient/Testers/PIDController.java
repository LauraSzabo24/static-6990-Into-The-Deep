package Ancient.Testers;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;


@Config
@TeleOp(name = "PID Test")
@Disabled
public class PIDController extends LinearOpMode {

    DcMotorEx motor1;
    DcMotorEx motor2;

    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.01;
    public static double Kd = 0.0;

    public static double targetPosition = 100;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {


        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        motor1 = hardwareMap.get(DcMotorEx.class, "slide1");
        motor2 = hardwareMap.get(DcMotorEx.class, "slide2");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int targetPosition = 0;
        waitForStart();




        while(opModeIsActive()){
            if(gamepad1.a) {
                targetPosition = 50;
            }
            if(gamepad1.b){
                targetPosition = -50;
            }
                double power = returnPower(targetPosition, motor1.getCurrentPosition());
                packet.put("power", power);
                packet.put("position", motor1.getCurrentPosition());
                packet.put("error", lastError);
                packet.put("targetPosition", targetPosition);
                telemetry.addData("power", power);
                telemetry.addData("position",motor1.getCurrentPosition());
                telemetry.addData("error", lastError);

                motor1.setPower(-power);
                motor2.setPower(power);
                telemetry.update();

                dashboard.sendTelemetryPacket(packet);
        }
    }

    public double returnPower(double reference, double state){
        double error = reference - state;
        double derivative = (error-lastError)/timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error* Kp) +  (derivative * Kd);
        return output;
    }
}
