package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Reicher Robotics on 11/28/2017.
 */

@TeleOp(name="LeagueChampionship", group="TeleOpRelic")
//@Disabled
public class LeagueChampionship extends LinearOpMode {

    HardwareRelic Chris = new HardwareRelic();
    ElapsedTime runtime = new ElapsedTime();

    private double leftFrontPower = 0.0;
    private double leftRearPower = 0.0;
    private double rightFrontPower = 0.0;
    private double rightRearPower = 0.0;

    private double drive = 0.0;
    private double strafe = 0.0;
    private double turn = 0.0;

    private double maxDrivePower = 0.0;

    private boolean FLAG_G2_START_PRESSED = false;
    private boolean FLAG_G2_DRIVE_ON = false;

    private boolean FLAG_G1A_PRESSED = false;
    private boolean FLAG_G2A_PRESSED = false;
    private boolean FLAG_CONVEYOR_ON = false;

    private boolean FLAG_G1Y_PRESSED = false;
    private boolean FLAG_G2Y_PRESSED = false;
    private boolean FLAG_PLATFORM_DOWN = true;

    private boolean FLAG_G1X_PRESSED = false;
    private boolean FLAG_G2X_PRESSED = false;
    private boolean FLAG_GRIPPER_OPEN = false;

    private boolean FLAG_G1BACK_PRESSED = false;
    private boolean FLAG_G2BACK_PRESSED = false;
    private boolean FLAG_ALIGN_UP = true;

    @Override
    public void runOpMode() {
        Chris.init(hardwareMap);
        Chris.stopUsingEncoders();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // Drive with joysticks
            joystickDrive();
            Chris.drive(leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

            // Conveyor with buttons
            conveyorButton();
            if(gamepad1.b || gamepad2.b){
                Chris.conveyorSpitOut();
                FLAG_CONVEYOR_ON = false;
            } else {
                if(FLAG_CONVEYOR_ON){
                    Chris.conveyorSuckIn();
                } else {
                    Chris.conveyorStop();
                }
            }

            // Platform with button
            platformButton();
            if(FLAG_PLATFORM_DOWN) {
                Chris.platformDown();
            } else {
                Chris.platformUp();
                FLAG_CONVEYOR_ON = false;
            }

            // Gripper with button
            gripperButton();
            if(FLAG_GRIPPER_OPEN){
                Chris.gripperClose();
                FLAG_CONVEYOR_ON = false;
            } else {
                Chris.gripperRelease();
            }

            // Relic Extension with buttons
            if(gamepad1.dpad_left || gamepad2.dpad_left) {
                Chris.relicExtExtend();
            } else if(gamepad1.dpad_right || gamepad2.dpad_right){
                Chris.relicExtRetract();
            } else {
                Chris.relicExtStop();
            }

            // Relic Wrist with buttons
            if(gamepad1.dpad_up || gamepad2.dpad_up) {
                Chris.relicWristUp();
            } else if(gamepad1.dpad_down || gamepad2.dpad_down){
                Chris.relicWristDown();
            } else {
                Chris.relicWristStop();
            }

            // Relic Claw with buttons
            if(gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                Chris.relicClawOpen();
            } else if(gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1){
                Chris.relicClawClose();
            }

            // Alignment with button
            alignButton();
            if(FLAG_ALIGN_UP){
                Chris.alignUp();
            } else {
                Chris.alignDown();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void joystickDrive(){
        if(gamepad1.left_bumper || gamepad2.left_bumper) {
            maxDrivePower = 0.3;
        } else if(gamepad1.right_bumper || gamepad2.right_bumper) {
            maxDrivePower = 0.5;
        } else {
            maxDrivePower = 0.7;
        }

        drive = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x;

        if (gamepad2.start) {
            FLAG_G2_START_PRESSED = true;
        } else if (!gamepad2.start && FLAG_G2_START_PRESSED) {
            if (FLAG_G2_DRIVE_ON) {
                FLAG_G2_DRIVE_ON = false;
            } else {
                FLAG_G2_DRIVE_ON = true;
            }
            FLAG_G2_START_PRESSED = false;
        }

        if(FLAG_G2_DRIVE_ON){
            drive = gamepad2.left_stick_y;
            strafe = -gamepad2.left_stick_x;
            turn = -gamepad2.right_stick_x;
        }

        rightFrontPower = Range.clip(drive - strafe - turn, -maxDrivePower, maxDrivePower);
        rightRearPower = Range.clip(drive - strafe + turn, -maxDrivePower, maxDrivePower);
        leftFrontPower = Range.clip(drive + strafe + turn, -maxDrivePower, maxDrivePower);
        leftRearPower = Range.clip(drive + strafe - turn, -maxDrivePower, maxDrivePower);
    }

    public void conveyorButton(){
        if (gamepad1.a) {
            FLAG_G1A_PRESSED = true;
        } else if (!gamepad1.a && FLAG_G1A_PRESSED) {
            if (FLAG_CONVEYOR_ON) {
                FLAG_CONVEYOR_ON = false;
            } else {
                FLAG_CONVEYOR_ON = true;
            }
            FLAG_G1A_PRESSED = false;
        }

        if (gamepad2.a) {
            FLAG_G2A_PRESSED = true;
        } else if (!gamepad2.a && FLAG_G2A_PRESSED) {
            if (FLAG_CONVEYOR_ON) {
                FLAG_CONVEYOR_ON = false;
            } else {
                FLAG_CONVEYOR_ON = true;
            }
            FLAG_G2A_PRESSED = false;
        }
    }

    public void platformButton(){
        if (gamepad1.y) {
            FLAG_G1Y_PRESSED = true;
        } else if (!gamepad1.y && FLAG_G1Y_PRESSED) {
            if (FLAG_PLATFORM_DOWN) {
                FLAG_PLATFORM_DOWN = false;
            } else {
                FLAG_PLATFORM_DOWN = true;
            }
            FLAG_G1Y_PRESSED = false;
        }

        if (gamepad2.y) {
            FLAG_G2Y_PRESSED = true;
        } else if (!gamepad2.y && FLAG_G2Y_PRESSED) {
            if (FLAG_PLATFORM_DOWN) {
                FLAG_PLATFORM_DOWN = false;
            } else {
                FLAG_PLATFORM_DOWN = true;
            }
            FLAG_G2Y_PRESSED = false;
        }
    }

    public void gripperButton(){
        if (gamepad1.x) {
            FLAG_G1X_PRESSED = true;
        } else if (!gamepad1.x && FLAG_G1X_PRESSED) {
            if (FLAG_GRIPPER_OPEN) {
                FLAG_GRIPPER_OPEN = false;
            } else {
                FLAG_GRIPPER_OPEN = true;
            }
            FLAG_G1X_PRESSED = false;
        }

        if (gamepad2.x) {
            FLAG_G2X_PRESSED = true;
        } else if (!gamepad2.x && FLAG_G2X_PRESSED) {
            if (FLAG_GRIPPER_OPEN) {
                FLAG_GRIPPER_OPEN = false;
            } else {
                FLAG_GRIPPER_OPEN = true;
            }
            FLAG_G2X_PRESSED = false;
        }
    }

    public void alignButton(){
        if (gamepad1.start) {
            FLAG_G1BACK_PRESSED = true;
        } else if (!gamepad1.start && FLAG_G1BACK_PRESSED) {
            if (FLAG_ALIGN_UP) {
                FLAG_ALIGN_UP = false;
            } else {
                FLAG_ALIGN_UP = true;
            }
            FLAG_G1BACK_PRESSED = false;
        }
    }
}
