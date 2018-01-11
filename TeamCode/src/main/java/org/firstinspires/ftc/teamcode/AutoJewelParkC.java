package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Reicher Robotics on 11/28/2017.
 */

@Autonomous(name = "Jewel&Park C", group = "AutoRelic")
//@Disabled
public class AutoJewelParkC extends LinearOpMode {
    HardwareRelic Chris = new HardwareRelic();

    @Override
    public void runOpMode() {
        Chris.init(hardwareMap);
        Chris.setFieldPosition('C');

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