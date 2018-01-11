package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Reicher Robotics on 11/28/2017.
 */

public class HardwareRelic {
    /*
     * MOTORS & SERVOS MEMBERS
     */
    // Drive Motors
    public DcMotor leftFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightRearDrive = null;
    // Conveyor Motors
    public DcMotor leftConveyor = null;
    public DcMotor rightConveyor = null;
    // Relic Extension Motor
    public DcMotor relicExtension = null;
    // Platform Servos
    public Servo leftPlatform = null;
    public Servo rightPlatform = null;
    public Servo gripper = null;
    // Stick Dropper Servo
    public Servo rightStickDropper = null;
    // Alignment Servo
    public Servo alignDropper = null;
    // Relic Grabber
    public CRServo relicWrist = null;
    public Servo relicClaw = null;

    /*
     * SENSOR MEMBERS
     */
    // IMU
    public BNO055IMU imu = null;
    //public Orientation angles = null;
    public Acceleration gravity = null;
    Orientation currAngles = null;
    float currHeading = (float) 0.0;
    float finalHeading = (float) 0.0;
    // Sensors
    public ColorSensor rightColor = null;
    public DigitalChannel alignTouch = null;
    public DigitalChannel teamColorSw = null;
    // Vuforia
    public VuforiaLocalizer vuforia;

    /*
     * OTHER MEMBERS
     */
    // Local Variables
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap hdM = null;
    private double leftPosition = 0.0;
    private double rightPosition = 1.0;
    private String teamColor = "";
    private char fieldPosition = 'e';
    private String fieldSide = "";
    private String fieldRow = "";
    private String jewelColor = "";
    private String pictographColumn = "";
    // Final Variables
    public final static int DRIVE_TICKS = 1120;
    public final static int CONVEYOR_TICKS = 2240;
    public final static double WHEEL_CIR = 18.85;
    public final static double WHEEL_FB_COF = 1.0;
    public final static double WHEEL_LR_COF = 0.6;

    /*
     * CONSTRUCTOR
     */
    public HardwareRelic() {
    }

    /*
     * HARDWARE MAPPINGS & MODE SETS
     */
    public void init(HardwareMap hwMap){
        hdM = hwMap;
        /*
         * HARDWARE MAPPINGS
         */
        // Drive Motors
        leftFrontDrive  = hdM.get(DcMotor.class, "lfDrive");
        leftRearDrive  = hdM.get(DcMotor.class, "lrDrive");
        rightFrontDrive = hdM.get(DcMotor.class, "rfDrive");
        rightRearDrive = hdM.get(DcMotor.class, "rrDrive");
        // Conveyor Motors
        leftConveyor = hdM.get(DcMotor.class, "lConveyor");
        rightConveyor = hdM.get(DcMotor.class, "rConveyor");
        // Relic Extension Motor
        relicExtension = hdM.get(DcMotor.class, "rExt");
        // Platform Servos
        leftPlatform = hdM.get(Servo.class, "lPlat");
        rightPlatform = hdM.get(Servo.class, "rPlat");
        gripper = hdM.get(Servo.class, "gripper");
        // Stick Dropper Servo
        rightStickDropper = hdM.get(Servo.class, "rSDrop");
        // Alignment Servo
        alignDropper = hdM.get(Servo.class, "aDrop");
        // Relic Servos
        relicWrist = hdM.get(CRServo.class, "rWrist");
        relicClaw = hdM.get(Servo.class, "rClaw");

        // Sensors
        rightColor = hdM.get(ColorSensor.class, "rColor");
        alignTouch = hdM.get(DigitalChannel.class, "aTouch");
        teamColorSw = hdM.get(DigitalChannel.class, "tcSw");

        /*
         * MODE SETS
         */
        // Drive Motors
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Conveyor Motors
        leftConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightConveyor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Relic Extension
        relicExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        relicExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alignTouch.setMode(DigitalChannel.Mode.INPUT);
        teamColorSw.setMode(DigitalChannel.Mode.INPUT);

        /*
         * DIRECTION SETS
         */
        // Drive Motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        // Conveyor Motors
        leftConveyor.setDirection(DcMotor.Direction.FORWARD);
        rightConveyor.setDirection(DcMotor.Direction.REVERSE);
        // Relic Extension Motor
        relicExtension.setDirection((DcMotor.Direction.REVERSE));
        // Relic Servos
        relicWrist.setDirection(CRServo.Direction.REVERSE);
        relicClaw.setDirection(Servo.Direction.FORWARD);

        /*
         * STARTS
         */
        // Drive Motors
        driveStop();
        // Conveyor Motors
        conveyorStop();
        // Platform Servos
        platformDown();
        // Gripper Servo
        gripperRelease();
        // Stick Dropper Servos
        stickZero();
        // Alignment Dropper Servo
        alignUp();

        setTeamColor();

        /*
         * IMU SETUP
         */
        // IMU
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "IMU";
        imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hdM.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // Vuforia
        int cameraMonitorViewId = hdM.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hdM.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = "AZVMQkn/////AAAAGWnTDWgAuUMtibgxaQdCLakPw1hQPWq7MN95sbj2X1XMV/H8y+W5Z0p4UcX2XcSFwc0keQu55qlUJSEZnWJVJGv1Wzc28zWrfXeBqNrlwTymybCBrswf4BAgfpYMAuuxLVwZp7bSomD4njAU6hWsD0yj4rqUSFX3O2K+0H9xUdwSoxcfv6uXbRhNMRAMWLdhbrU+rX6fW3XqdC/kR1biAK7VVmSFKkKF9oZ4Y6j+ym5UM3/Oo+NOz3Algl4hx/sjjgkv7ZeEHiFIR0PMSw5/spoZE6yfqyzh2Ozz/o+2BpZH2MS7lbVr6g9JbzU1yBNLhLeId/8VkvdfRYiNMsrVA/W2iyNL80nBaE5dke3ydmS5";
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters);
    }

    public void stopUsingEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startHeading(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }

    public void readPictograph(){
        pictographColumn = "UNKNOWN";
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        int ctr = 0;
        while (ctr < 80) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            //if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                 pictographColumn = vuMark.toString();
                 //ctr = 100;
            //}
            runtime.reset();
            while(runtime.time() < 0.05){
            }
            ctr++;
        }
    }

    public String getPictographColumn(){
        return pictographColumn;
    }

    public char getFieldPosition(){
        return fieldPosition;
    }

    public void setFieldPosition(char position){
        if (position == 'A' || position == 'a'){
            fieldPosition = 'A';
            fieldSide = "RIGHT";
            fieldRow = "FRONT";
        }
        if (position == 'B' || position == 'b'){
            fieldPosition = 'B';
            fieldSide = "RIGHT";
            fieldRow = "BACK";
        }
        if (position == 'C' || position == 'c'){
            fieldPosition = 'C';
            fieldSide = "LEFT";
            fieldRow = "BACK";
        }
        if (position == 'D' || position == 'd'){
            fieldPosition = 'D';
            fieldSide = "LEFT";
            fieldRow = "FRONT";
        }
    }

    public String getFieldSide(){
        return fieldSide;
    }

    public void setFieldSide(String side){
        if (side.charAt(0) == 'L' || side.charAt(0) == 'l'){
            fieldSide = "LEFT";
        }
        if (side.charAt(0) == 'R' || side.charAt(0) == 'r'){
            fieldSide = "RIGHT";
        }
    }

    public boolean sideIsLeft(){
        if(fieldSide.equals("LEFT")){
            return true;
        }
        return false;
    }

    public boolean sideIsRight(){
        if(fieldSide.equals("RIGHT")){
            return true;
        }
        return false;
    }

    public String getFieldRow(){
        return fieldRow;
    }

    public void setFieldRow(String row){
        if (row.charAt(0) == 'F' || row.charAt(0) == 'f'){
            fieldRow = "FRONT";
        }
        if (row.charAt(0) == 'B' || row.charAt(0) == 'b'){
            fieldRow = "BACK";
        }
    }

    public boolean rowIsFront(){
        if(fieldRow.equals("FRONT")){
            return true;
        }
        return false;
    }

    public boolean rowIsBack(){
        if(fieldRow.equals("BACK")){
            return true;
        }
        return false;
    }

    public String getTeamColor(){
        return teamColor;
    }

    private void setTeamColor(){
        if (teamColorSw.getState()){
            teamColor = "RED";
        } else {
            teamColor = "BLUE";
        }
    }

    public String getJewelColor(){
        return jewelColor;
    }

    public void setJewelColor(){
        if(rightColor.red() > rightColor.blue()){
            jewelColor = "RED";
        } else {
            jewelColor = "BLUE";
        }
    }

    public boolean compareTeamColorToJewelColor(){
        return teamColor.equals(jewelColor);
    }

    public boolean isAligned(){
        return alignTouch.getState();
    }

    /*
     * DRIVE METHODS
     */
    // Drive Method
    // Drive motors have individual powers
    public void drive(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower){
        leftFrontDrive.setPower(-leftFrontPower);
        leftRearDrive.setPower(-leftRearPower);
        rightFrontDrive.setPower(-rightFrontPower);
        rightRearDrive.setPower(-rightRearPower);
    }

    // Drive Forward
    // Drive motors all move forward with the same power
    public void driveForward(double power){
        drive(power, power, power, power);
    }

    // Drive Backward
    // Drive motors all move backward with the same power
    public void driveBackward(double power){
        drive(power, power, power, power);
    }

    // Drive Stop
    // Drive motors all stop
    public void driveStop(){
        drive(0.0, 0.0, 0.0, 0.0);
    }

    // Drive Forward a Certain Distance
    // Drive motors all move forward a certain distance with the same power
    public void driveForwardDistance(double power, double distance){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) (distance / WHEEL_CIR * WHEEL_FB_COF * DRIVE_TICKS);

        leftFrontDrive.setTargetPosition(ticks);
        leftRearDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);
        rightRearDrive.setTargetPosition(ticks);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()){
        }

        driveStop();

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Drive Backward a Certain Distance
    // Drive motors all move backward a certain distance with the same power
    public void driveBackwardDistance(double power, double distance){
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) (distance / WHEEL_CIR * WHEEL_FB_COF * DRIVE_TICKS);

        leftFrontDrive.setTargetPosition(-ticks);
        leftRearDrive.setTargetPosition(-ticks);
        rightFrontDrive.setTargetPosition(-ticks);
        rightRearDrive.setTargetPosition(-ticks);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveBackward(power);

        while(leftFrontDrive.isBusy() && leftRearDrive.isBusy() && rightFrontDrive.isBusy() && rightRearDrive.isBusy()){
        }

        driveStop();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Turn Right
    // Drive motors turn the robot to the right with the same power
    public void turnRight(double power){
        drive(-power, -power, power, power);
    }

    // Turn Left
    // Drive motors turn the robot to the left with the same power
    public void turnLeft(double power){
        drive(power, power, -power, -power);
    }

    public void turnRightAngle(double power, int angle){

        currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        finalHeading = currHeading - angle;

        turnRight(power);

        while(currHeading > finalHeading){
            currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }

        driveStop();
    }

    public void turnLeftAngle(double power, int angle){

        currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        finalHeading = currHeading + angle;

        turnLeft(power);

        while(currHeading < finalHeading){
            currHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }

        driveStop();
    }

    public void driveRight(double power){
        drive(-power, power, power, -power);
    }

    public void driveLeft(double power){
        drive(power, -power, -power, power);
    }

    public void moveFromBalanceToSafe(){
        if(rowIsFront()) {
            driveForwardDistance(0.5, 41.0);
            if(sideIsLeft()) {
                turnRightAngle(0.4, 85);
            } else {
                turnLeftAngle(0.4, 85);
            }
            driveBackwardDistance(0.5, 14.0);
        } else {
            driveForwardDistance(0.8, 24.0);
            if(sideIsLeft()) {
                turnRightAngle(0.4, 85);
                driveForwardDistance(0.8, 13.0);
                turnRightAngle(0.4, 60);
            } else {
                turnLeftAngle(0.4, 85);
                driveForwardDistance(0.8, 13.0);
                turnLeftAngle(0.4, 60);
            }
        }
    }

    public void moveFromBalanceToSpecificColumn(){
        gripperClose();
        if(sideIsRight()) {
            driveForwardDistance(0.8, columnDistance());
            turnLeftAngle(0.4, 85);
            alignDown();
            driveBackwardDistance(0.5, 7.0);
            driveRight(0.4);
            runtime.reset();
            while(!isAligned() && runtime.time() < 3.0){
            }
            driveStop();
            alignUp();
            platformUp();
            runtime.reset();
            while(runtime.time() < 1.0){
            }
            gripperRelease();
            runtime.reset();
            while(runtime.time() < 0.5){
            }
            platformDown();
            driveForwardDistance(0.5, 4.0);
            driveBackwardDistance(0.5, 8.0);
            driveForwardDistance(0.5, 8.0);
        } else {
            driveBackwardDistance(0.8, columnDistance());
            turnLeftAngle(0.4, 85);
            alignDown();
            driveBackwardDistance(0.5, 7.0);
            driveRight(0.4);
            runtime.reset();
            while(!isAligned() && runtime.time() < 2.0){
            }
            driveStop();
            alignUp();
            platformUp();
            runtime.reset();
            while(runtime.time() < 1.0){
            }
            gripperRelease();
            runtime.reset();
            while(runtime.time() < 0.5){
            }
            platformDown();
            driveForwardDistance(0.5, 4.0);
            driveBackwardDistance(0.5, 8.0);
            driveForwardDistance(0.5, 8.0);
        }
    }

    public double columnDistance(){
        double distance = 0;
        if (sideIsRight()) {
            if (pictographColumn.equals("RIGHT")) {
                distance = 23.5;
            } else if (pictographColumn.equals("CENTER")) {
                distance = 31;
            } else {
                distance = 38.5;
            }
        } else {
            if (pictographColumn.equals("RIGHT")) {
                distance = 42.5;
            } else if (pictographColumn.equals("CENTER")) {
                distance = 35.0;
            } else {
                distance = 27.5;
            }
        }
        return distance;
    }

    public void conveyorRun(double power){
        leftConveyor.setPower(power);
        rightConveyor.setPower(power);
    }

    public void conveyorSuckIn(){
        conveyorRun(0.95);
    }

    public void conveyorStop(){
        conveyorRun(0.0);
    }

    public void conveyorSpitOut(){
        conveyorRun(-0.95);
    }

    public void stickMove(double rightPosition){
        rightStickDropper.setPosition(rightPosition);
    }

    public void stickZero(){
        stickMove(0.0);
    }

    public void stickLower(){
        if(fieldPosition == 'A' || fieldPosition == 'B' || fieldPosition == 'D'){
            stickMove(1.0);
        } else {
            stickMove(0.0);
        }
    }

    public void stickRaise(){
        stickMove(0.0);
    }

    public void removeOtherColorJewel(){
        stickLower();
        runtime.reset();
        while(runtime.time() < 0.5){
        }
        setJewelColor();
        if (fieldPosition == 'A' || fieldPosition == 'B' || fieldPosition == 'D') {
            if(compareTeamColorToJewelColor()) {
                turnLeftAngle(0.2, 7);
                stickRaise();
                turnRightAngle(0.2, 7);
            } else {
                turnRightAngle(0.2, 7);
                stickRaise();
                turnLeftAngle(0.2, 7);
            }
        } else {
            if(compareTeamColorToJewelColor()) {
                turnLeftAngle(0.2, 7);
                stickRaise();
                turnRightAngle(0.2, 7);
            } else {
                turnRightAngle(0.2, 7);
                stickRaise();
                turnLeftAngle(0.2, 7);
            }
        }
    }

    public void platformLower(double stepSize){
        rightPosition = rightPlatform.getPosition() - stepSize;
        if(rightPosition < 0.0) {
            rightPosition = 0.0;
        }
        leftPosition = leftPlatform.getPosition() + stepSize;
        if(leftPosition > 1.0) {
            leftPosition = 1.0;
        }
        rightPlatform.setPosition(rightPosition);
        leftPlatform.setPosition(leftPosition);
    }

    public void platformRaise(double stepSize){
        rightPosition = rightPlatform.getPosition() + stepSize;
        if(rightPosition > 1.0) {
            rightPosition = 1.0;
        }
        leftPosition = leftPlatform.getPosition() - stepSize;
        if(leftPosition < 0.0) {
            leftPosition = 0.0;
        }
        rightPlatform.setPosition(rightPosition);
        leftPlatform.setPosition(leftPosition);
    }

    public void platformDown(){
        rightPlatform.setPosition(0.0);
        leftPlatform.setPosition(1.0);
    }

    public void platformUp(){
        rightPlatform.setPosition(1.0);
        leftPlatform.setPosition(0.0);
    }

    public void gripperOpen(){
        gripper.setPosition(1.0);
    }

    public void gripperClose(){
        gripper.setPosition(0.0);
    }

    public void gripperRelease(){
        gripper.setPosition(0.5);
    }

    public void alignUp(){
        alignDropper.setPosition(0.0);
    }

    public void alignDown(){
        alignDropper.setPosition(1.0);
    }

    public void relicExtExtend(){
        relicExtension.setPower(1.0);
    }

    public void relicExtRetract(){
        relicExtension.setPower(-1.0);
    }

    public void relicExtStop(){
        relicExtension.setPower(0.0);
    }

    public void relicWristUp(){
        relicWrist.setPower(0.75);
    }

    public void relicWristDown(){
        relicWrist.setPower(-0.75);
    }

    public void relicWristStop(){
        relicWrist.setPower(0.0);
    }

    public void relicClawOpen(){
        relicClaw.setPosition(0.0);
    }

    public void relicClawClose(){
        relicClaw.setPosition(1.0);
    }
}
