package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Reicher Robotics on 11/28/2017.
 */

@Autonomous(name = "Jewel&Park D", group = "AutoRelic")
//@Disabled
public class AutoJewelParkD extends LinearOpMode {
    HardwareRelic Chris = new HardwareRelic();

    @Override
    public void runOpMode() {
        Chris.init(hardwareMap);
        Chris.setFieldPosition('D');

        telemetry.addLine("Team Color: " + Chris.getTeamColor());
        telemetry.addLine("Field Position: " + Chris.getFieldPosition());
        telemetry.addLine("Field Side: " + Chris.getFieldSide());
        telemetry.addLine("Field Row: " + Chris.getFieldRow());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for the start button to be pressed.
        waitForStart();

        Chris.gripperClose();
        Chris.startHeading();
        Chris.removeOtherColorJewel();
        Chris.moveFromBalanceToSafe();
    }
}