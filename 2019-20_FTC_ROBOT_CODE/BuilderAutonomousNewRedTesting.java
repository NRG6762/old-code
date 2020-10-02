package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

//Robot is placed in on the wall exactly between side wall and Skybridge tape

/*
 * NRG6762
 * Northwestern Regional 7 Gearheads
 * 2019-2020 Season - Skystone
 * Autonomous Class - Base
 * Written by Aiden Maraia
 * Version: 2/21/2020
 * Feel free to make any changes and use at your disposal.
 */
@Autonomous(name="Testing: Builder Autonomous - New Red", group="Testing")
//@Disabled
public class BuilderAutonomousNewRedTesting extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private BuilderHardware robot = new BuilderHardware(true);

    //Declare the use of mecanum telemetry
    private boolean mecanumTelemetry    = true;
    private boolean firstTime = true;
    private boolean firstTimeStone = true;
    private boolean skystoneSeen = false;
    private double skystoneLocation = 0.0;
    private OpenGLMatrix lastLocation;

    public String driveStep = "Start";
    public String stoneStep = "Start";
    public String visionStep = "Init Vision";

    private Orientation angles;
    private double lastOrient;
    private double currOrient;
    private double startOrient;
    private double pureOrient;
    private long rotationCount = 0;

    private double ticksHold    = 0;
    private double distanceHold = 0;
    private double colorHold    = 0;

    private double holdTicksScissor = 0;
    private double holdTicksLinear = 0;

    private boolean frontLimitState = false;
    private boolean backLimitState = false;
    private boolean upperLimitState = false;
    private boolean lowerLimitState = false;

    private int foundationDistanceHold = 8;

    /* Code to be run while the robot is running (from pressing play to stop) */
    @Override
    public void runOpMode() {

        //Initialize the Hardware Map
        robot.init(hardwareMap);

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        //Signify the Hardware Map has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Use Drive", robot.driveMotorsTrue);
        telemetry.addData("Use Scissor", robot.scissorDriveTrue);

        //Update the telemetry
        telemetry.update();

        robot.colorLight.enable(false);

        //Wait for the Pressing of the Start Button
        waitForStart();

        //Reset the game clock to zero in Start()
        runtime.reset();

        robot.flipper.setPosition(robot.flipperOpen);

        startOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            pureOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - startOrient;

            currOrient = pureOrient;

            if((pureOrient - lastOrient) > 5){
                rotationCount++;
            }else if((pureOrient - lastOrient) < -5){
                rotationCount--;
            }

            currOrient = pureOrient + rotationCount * 2 * Math.PI;

            upperLimitState = robot.limitUpper.getState();
            lowerLimitState = robot.limitLower.getState();
            frontLimitState = robot.limitFront.getState();
            backLimitState = robot.limitBack.getState();


            switch (driveStep) {

                case "Start":

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    break;

                case "To Stones 1":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.leftFront.getCurrentPosition(), robot.ticksPerInchFront * 12, 1),
                            0, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        visionStep = "End Vision";

                        robot.gripper.setPosition(robot.gripperOpen);

                        driveStep = "To Stones 2";

                        firstTime = true;
                    }

                    break;

                case "To Stones 1.5":

                    if(skystoneSeen){

                        driveStep = "To Stones 2";

                    }

                    break;

                case "To Stones 2":

                    if(firstTime){

                        distanceHold = robot.distanceFront.getDistance(DistanceUnit.INCH);

                        if(distanceHold > 5){
                            distanceHold = 5;
                        }

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.leftFront.getCurrentPosition(), robot.ticksPerInchFront * distanceHold, .6),
                            0, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        if(skystoneLocation > 1){
                            driveStep = "Crab Along Stones Right";
                        }else if(skystoneLocation < -1){
                            driveStep = "Crab Along Stones Left";
                        }else{
                            stoneStep = "Forward";
                            driveStep = "Hold";
                        }

                        firstTime = true;
                    }

                    break;

                case "Crab Along Stones Right":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;
                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.leftFront.getCurrentPosition(), robot.ticksPerInchSide * 8, 1),
                            Math.PI / 2, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        stoneStep = "Forward";
                        driveStep = "Hold";

                        foundationDistanceHold += 8;

                        firstTime = true;
                    }

                    break;

                case "Crab Along Stones Left":

                    if(firstTime){

                        ticksHold = robot.rightFront.getCurrentPosition();

                        firstTime = false;
                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.rightFront.getCurrentPosition(), robot.ticksPerInchSide * 8, 1),
                            -Math.PI / 2, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        stoneStep = "Forward";
                        driveStep = "Hold";

                        foundationDistanceHold -= 8;

                        firstTime = true;
                    }

                    break;

                /*case "Speen":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(0,
                            0, twist(currOrient, Math.PI / 2))){

                        stopAndResetMotors();

                        driveStep = "To Foundation";

                        firstTime = true;
                    }

                    break;*/

                case "To Foundation":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.leftFront.getCurrentPosition(), robot.ticksPerInchSide * (96 - foundationDistanceHold), 1),
                            Math.PI / 2, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        stoneStep = "Uppy";
                        driveStep = "Drop Stone";

                        firstTime = true;
                    }

                    break;

                /*case "Respeen":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(0,
                            0, twist(currOrient, -Math.PI / 2))){

                        stopAndResetMotors();

                        stoneStep = "Down & Drop";

                        firstTime = true;
                    }

                    break;*/

                case "Drop Stone":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.leftFront.getCurrentPosition(), robot.ticksPerInchFront * 12, 1),
                            0, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        visionStep = "End Vision";

                        robot.gripper.setPosition(robot.gripperOpen);

                        driveStep = "Move Foundation";

                        firstTime = true;
                    }

                    break;

                case "Crab Return":

                    if(firstTime){

                        ticksHold = robot.rightFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.rightFront.getCurrentPosition(), robot.ticksPerInchSide * 96, 1),
                            -Math.PI / 2, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        driveStep = "Grab Stone";

                        firstTime = true;

                    }

                    break;

                case "Move Foundation":

                    if(firstTime){

                        ticksHold = robot.rightFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.rightFront.getCurrentPosition(), robot.ticksPerInchFront * 24, 1),
                            Math.PI, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        stoneStep = "Release";

                        firstTime = true;

                    }

                    break;

                case "To Skybridge":

                    if(firstTime){

                        robot.colorLight.enable(true);

                        colorHold = robot.color.red();

                        ticksHold = robot.rightFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.rightFront.getCurrentPosition(), robot.ticksPerInchSide * 48, 1),
                            -Math.PI / 2, -correction(startOrient, currOrient)) || Math.abs(colorHold - robot.color.red()) > 20){

                        stopAndResetMotors();

                        driveStep = "Park";

                        robot.colorLight.enable(false);

                        firstTime = true;

                    }

                    break;

                case "Hold":

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    break;

                case "Park":

                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);

                    break;

            }

            switch(stoneStep){

                case "Start":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();

                        firstTimeStone = false;
                    }

                    if(scissorControl(false, robot.rotatorMidPosition, 10.5, 0)){
                        distanceHold = robot.distanceFront.getDistance(DistanceUnit.INCH);
                        driveStep = "To Stones 1";
                        stoneStep = "Start B";
                        firstTimeStone = true;
                    }

                    break;

                case "Start B":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();

                        firstTimeStone = false;
                    }

                    if(scissorControl(false, robot.rotatorMidPosition, robot.gripperClearing, 0)){
                        distanceHold = robot.distanceFront.getDistance(DistanceUnit.INCH);
                        stoneStep = "Hold";
                        firstTimeStone = true;
                    }

                    break;

                case "Forward":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(false, robot.rotatorMidPosition, robot.gripperDown, 5)){

                        stoneStep = "Reverse";

                        robot.gripper.setPosition(robot.gripperClosed);

                        firstTimeStone = true;

                    }

                    break;

                case "Reverse":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(true, robot.rotatorMidPosition, robot.gripperDown, 0)){

                        stoneStep = "Hold";

                        driveStep = "To Foundation";

                        robot.gripper.setPosition(robot.gripperClosed);

                        firstTimeStone = true;

                    }

                    break;

                case "Uppy":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(true, robot.rotatorMidPosition, robot.gripperDown + 2, 0)){
                        stoneStep = "Hold";
                        firstTimeStone = true;
                    }

                    break;

                case "Down & Drop":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(true, robot.rotatorMidPosition, robot.gripperDown + 1, 0)){
                        robot.gripper.setPosition(robot.gripperOpen);
                        stoneStep = "Bottom Out";
                        driveStep = "Drop Stone";
                        firstTimeStone = true;
                    }

                    break;


                case "Bottom Out":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(false, robot.rotatorMidPosition, robot.gripperDown, 0)){
                        stoneStep = "Hold";
                        firstTimeStone = true;
                    }

                    break;

                case "Release":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(true, robot.rotatorMidPosition, robot.gripperDown + 3, 5)){
                        robot.gripper.setPosition(robot.gripperOpen);
                        stoneStep = "Back Out";
                        driveStep = "To Skybridge";
                        firstTimeStone = true;
                    }

                    break;


                case "Back Out":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(false, robot.rotatorMidPosition, robot.gripperDown, 0)){
                        stoneStep = "Hold";
                        firstTimeStone = true;
                    }

                    break;

                case "Hold":

                    robot.scissor.setPower(0.0);
                    robot.scissor2.setPower(0.0);

                    robot.linear.setPower(0.0);

                    break;
            }

            switch(visionStep){

                case "Init Vision":
                    robot.targetsSkyStone.activate();
                    visionStep = "Find Skystone";
                    break;

                case "Find Skystone":

                    // check all the trackable targets to see which one (if any) is visible.
                    skystoneSeen = false;
                    if (((VuforiaTrackableDefaultListener)robot.stoneTarget.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", robot.stoneTarget.getName());
                        skystoneSeen = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)robot.stoneTarget.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }

                    }

                    // Provide feedback as to where the robot is located (if we know).
                    if (skystoneSeen) {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        skystoneLocation = translation.get(1) / robot.mmPerInch;
                        telemetry.addData("Skystone Location", skystoneLocation);
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                    }

                    break;

                case "End Vision":
                    robot.targetsSkyStone.deactivate();
                    break;

            }

            telemetry.addData("Drive Step", driveStep);
            telemetry.addData("Stone Step", stoneStep);
            telemetry.addData("Front Distance", robot.distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("Scissor Distance", robot.distanceScissor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Current Orientation", currOrient);

            lastOrient = pureOrient;

            //Update the Telemetry
            telemetry.update();

        }

    }

    private boolean newDriveCommand(double polarSpeed, double polarAngle, double polarSpeen){

        //Change speed, angle, and spin into power levels for the four drive motors
        double leftFrontValue = polarSpeed * Math.sin(polarAngle + (Math.PI/4)) + polarSpeen;
        double rightFrontValue = polarSpeed * Math.cos(polarAngle + (Math.PI/4)) - polarSpeen;
        double leftBackValue = polarSpeed * Math.cos(polarAngle + (Math.PI/4)) + polarSpeen;
        double rightBackValue = polarSpeed * Math.sin(polarAngle + (Math.PI/4)) - polarSpeen;

        //Set maxValue to the absolute value of first power level
        double maxValue = Math.abs(leftFrontValue);

        //If the absolute value of the second power level is less than the maxValue, make it the new maxValue
        if(Math.abs(rightFrontValue) > maxValue){maxValue = Math.abs(rightFrontValue);}

        //If the absolute value of the third power level is less than the maxValue, make it the new maxValue
        if(Math.abs(leftBackValue) > maxValue){maxValue = Math.abs(leftBackValue);}

        //If the absolute value of the fourth power level is less than the maxValue, make it the new maxValue
        if(Math.abs(rightBackValue) > maxValue){maxValue = Math.abs(rightBackValue);}

        //Check if need to scale -- if not set maxValue to 1 to nullify scaling
        if (maxValue < 1){maxValue = 1;}

        //Send the scaled power levels to the returned variable
        robot.leftFront.setPower(leftFrontValue / maxValue);
        robot.rightFront.setPower(rightFrontValue / maxValue);
        robot.leftBack.setPower(leftBackValue / maxValue);
        robot.rightBack.setPower(rightBackValue / maxValue);

        /* Mecanum Telemetry for troubleshooting */
        if(mecanumTelemetry) {

            //Show the calculated vector values
            /*telemetry.addData("Polar Speed", polarSpeed);
            telemetry.addData("Polar Angle", polarAngle);
            telemetry.addData("Polar Speen", polarSpeen);*/

            //Show the calculated, unscaled values
            telemetry.addData("Left Front Calculated Value", leftFrontValue);
            telemetry.addData("Right Front Calculated Value", rightFrontValue);
            telemetry.addData("Left Back Calculated Value", leftBackValue);
            telemetry.addData("Right Back Calculated Value", rightBackValue);

            //Show the scaling values
            telemetry.addData("Scaling Value", maxValue);

            telemetry.addData("Left Front Position", robot.leftFront.getCurrentPosition());
            telemetry.addData("Right Front Position", robot.rightFront.getCurrentPosition());
            telemetry.addData("Left Back Position", robot.leftBack.getCurrentPosition());
            telemetry.addData("Right Back Position", robot.rightBack.getCurrentPosition());

            //Show the calculated and scaled values
            /*telemetry.addData("Left Front Actual Value", finalReturns[0]);
            telemetry.addData("Right Front Actual Value", finalReturns[1]);
            telemetry.addData("Left Back Actual Value", finalReturns[2]);
            telemetry.addData("Right Back Actual Value", finalReturns[3]);*/

        }

        return polarSpeed < 0.01;

    }

    /*private double[] driveCommand(double startTicks, double currTicks, double goalTicks, double polarAngle, double startOrient, double currOrient, double goalOrient){

        double finalReturns[] = new double[4];

        double polarSpeed = motorCurve(startTicks, currTicks, goalTicks, 1);

        double polarSpeen = 0;

        if(startOrient == goalOrient){

            polarSpeen = 0;

        }else{

            polarSpeen = motorCurve(startOrient, currOrient, goalOrient, 1) / 3;

        }

        //Change speed, angle, and spin into power levels for the four drive motors
        double leftFrontValue = polarSpeed * Math.sin(polarAngle + (Math.PI/4)) + polarSpeen;
        double rightFrontValue = polarSpeed * Math.cos(polarAngle + (Math.PI/4)) - polarSpeen;
        double leftBackValue = polarSpeed * Math.cos(polarAngle + (Math.PI/4)) + polarSpeen;
        double rightBackValue = polarSpeed * Math.sin(polarAngle + (Math.PI/4)) - polarSpeen;

        //Set maxValue to the absolute value of first power level
        double maxValue = Math.abs(leftFrontValue);

        //If the absolute value of the second power level is less than the maxValue, make it the new maxValue
        if(Math.abs(rightFrontValue) > maxValue){maxValue = Math.abs(rightFrontValue);}

        //If the absolute value of the third power level is less than the maxValue, make it the new maxValue
        if(Math.abs(leftBackValue) > maxValue){maxValue = Math.abs(leftBackValue);}

        //If the absolute value of the fourth power level is less than the maxValue, make it the new maxValue
        if(Math.abs(rightBackValue) > maxValue){maxValue = Math.abs(rightBackValue);}

        //Check if need to scale -- if not set maxValue to 1 to nullify scaling
        if (maxValue < 1){maxValue = 1;}

        //Send the scaled power levels to the returned variable
        finalReturns[0] = leftFrontValue / maxValue;
        finalReturns[1] = rightFrontValue / maxValue;
        finalReturns[2] = leftBackValue / maxValue;
        finalReturns[3] = rightBackValue / maxValue;

        /* Mecanum Telemetry for troubleshooting
        if(mecanumTelemetry) {

            //Show the calculated vector values
            /*telemetry.addData("Polar Speed", polarSpeed);
            telemetry.addData("Polar Angle", polarAngle);
            telemetry.addData("Polar Speen", polarSpeen);

            //Show the calculated, unscaled values
            telemetry.addData("Left Front Calculated Value", leftFrontValue);
            telemetry.addData("Right Front Calculated Value", rightFrontValue);
            telemetry.addData("Left Back Calculated Value", leftBackValue);
            telemetry.addData("Right Back Calculated Value", rightBackValue);

            //Show the scaling values
            telemetry.addData("Scaling Value", maxValue);

            //Show the calculated and scaled values
            telemetry.addData("Left Front Actual Value", finalReturns[0]);
            telemetry.addData("Right Front Actual Value", finalReturns[1]);
            telemetry.addData("Left Back Actual Value", finalReturns[2]);
            telemetry.addData("Right Back Actual Value", finalReturns[3]);

            //Update telemetry
            telemetry.update();

        }

        return finalReturns;

    }*/

    /*private double heightToTicks(double desiredHeight){

        double heightToWidth = Math.sqrt(121 - Math.pow(desiredHeight / 5, 2));

        return heightToWidth * robot.scissorRevPerInch * robot.ticksPerScissorInch;

    }*/

    private double motorCurve(double startTicks, double currTicks, double goalTicks, double maxPower){

        double rangeTicks = goalTicks - startTicks;

        double shiftedCurrTicks = currTicks - startTicks;

        double curve = -4 * maxPower * (shiftedCurrTicks - rangeTicks) * (shiftedCurrTicks + 0.05 * rangeTicks) / (rangeTicks * rangeTicks);

        if(curve >= 1){
            curve = 1;
        }

        return curve;

    }

    private void stopAndResetMotors(){

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

    }

    private boolean scissorControl(boolean grip, double rotatorPosition, double scissorHeight, double linearPosition){

        if(grip){
            robot.gripper.setPosition(robot.gripperClosed);
        }else{
            robot.gripper.setPosition(robot.gripperOpen);
        }

        //robot.rotator.setPosition(rotatorPosition);

        double currHeight = robot.distanceScissor.getDistance(DistanceUnit.INCH);

        if(scissorHeight - currHeight > 0 && Math.abs(scissorHeight - currHeight) > robot.distanceRange) {
            robot.scissor.setPower(-1);
            robot.scissor2.setPower(-1);
        }else if(scissorHeight - currHeight < 0 && Math.abs(scissorHeight - currHeight) > robot.distanceRange) {
            robot.scissor.setPower(1);
            robot.scissor2.setPower(1);
        }else{
            robot.scissor.setPower(0);
            robot.scissor2.setPower(0);
        }

        if(!lowerLimitState || scissorHeight == 0 || Math.abs(currHeight - scissorHeight) < robot.distanceRange){
            robot.scissor.setPower(0);
            robot.scissor2.setPower(0);
        }

        if(!lowerLimitState || scissorHeight == 0){
            return true;
        }

        linearPosition *= robot.ticksPerLinearInch;

        double linearPower = 0;

        if((linearPosition - robot.linear.getCurrentPosition()) <= -20 && backLimitState){
            linearPower = 0.5;
        }else if((linearPosition - robot.linear.getCurrentPosition()) >= 20 && frontLimitState){
            linearPower = -0.5;
        }else{
            linearPower = 0.0;
        }

        robot.linear.setPower(linearPower);

        return (Math.abs(currHeight - scissorHeight) < robot.distanceRange || lowerLimitState) && (linearPower == 0);

    }

    public double correction(double oldHeading, double newHeading){

        double corrSpin = 0.0;
        if(Math.abs(oldHeading - newHeading) > Math.PI) {
            if (oldHeading > newHeading) {
                corrSpin = newHeading - oldHeading;
            } else if (oldHeading < newHeading) {
                corrSpin = oldHeading - newHeading;
            } else if (oldHeading == newHeading) {
                corrSpin = 0.0;
            }
        }
        return  corrSpin;

    }

    /*public double twist(double currHeading, double addHeading){

        double goalHeading = currHeading + addHeading;

        double speed = 1;

        if(addHeading < 0){
            speed = -1;
        }

        if(Math.abs(currHeading - goalHeading) < .05){
            speed = 0;
        }else if(Math.abs(currHeading - goalHeading) < Math.PI / 8){
            speed /= 4;
        }

        return speed;

    }*/

}
