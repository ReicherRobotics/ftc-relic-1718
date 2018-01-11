package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Reicher Robotics on 11/28/2017.
 */

@Autonomous(name = "Jewel&Dump&Park D", group = "AutoRelic")
//@Disabled
public class AutoJewelDumpParkD extends LinearOpMode {
    HardwareRelic Chris = new HardwareRelic();
    ElapsedTime runtime = new ElapsedTime();

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

        Chris.readPictograph();
        Chris.gripperClose();
        telemetry.addLine("Pictograph: " + Chris.getPictographColumn());
        telemetry.addLine("Column Distance: " + Chris.columnDistance());
        telemetry.update();
        Chris.startHeading();
        Chris.removeOtherColorJewel();
        Chris.moveFromBalanceToSpecificColumn();
    }
}