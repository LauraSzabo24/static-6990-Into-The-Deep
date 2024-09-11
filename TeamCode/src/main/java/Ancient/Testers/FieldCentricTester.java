package Ancient.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Auto.Mailbox;

@TeleOp
@Disabled
public class FieldCentricTester extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        Mailbox mail =  new Mailbox();

        /*while(true) {
            if (Mailbox.autoEndHead > 45 && Mailbox.autoEndHead < 135) {
                telemetry.addData("90 degrees", Mailbox.autoEndHead);
            } else if (Mailbox.autoEndHead > 135 && Mailbox.autoEndHead < 225) {
                telemetry.addData("180 degrees", Mailbox.autoEndHead);
            } else if (Mailbox.autoEndHead > 225 && Mailbox.autoEndHead < 315) {
                telemetry.addData("270 degrees", Mailbox.autoEndHead);
            } else if (Mailbox.autoEndHead > 315 || Mailbox.autoEndHead < 45) {
                telemetry.addData("360 degrees", Mailbox.autoEndHead);
            } else {
                telemetry.addData("idk", Mailbox.autoEndHead);
            }
            telemetry.update();
        }*/
    }
}
