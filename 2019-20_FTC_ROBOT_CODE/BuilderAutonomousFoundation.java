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
@Autonomous(name="Testing: Builder Autonomous - Foundation", group="Testing")
//@Disabled
public class BuilderAutonomousFoundation extends LinearOpMode {

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
    private double holdTime = 0;

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

                case "To Foundation":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.leftFront.getCurrentPosition(), robot.ticksPerInchFront * 26, 1),
                            0, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        driveStep = "Along Foundation";

                        firstTime = true;
                    }

                    break;

                case "Along Foundation":

                    if(firstTime){

                        ticksHold = robot.rightFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.rightFront.getCurrentPosition(), robot.ticksPerInchSide * 12, 1),
                            -Math.PI / 2, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        stoneStep = "Grab";

                        driveStep = "Hold";

                        firstTime = true;
                    }

                    break;

                case "Backwards":

                    if(firstTime){

                        ticksHold = -robot.rightFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, -robot.rightFront.getCurrentPosition(), robot.ticksPerInchFront * 34, .8),
                            -Math.PI, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        stoneStep = "Release";

                        driveStep = "Hold";

                        firstTime = true;
                    }

                    break;

                case "To Skybridge":

                    if(firstTime){

                        ticksHold = robot.leftFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.leftFront.getCurrentPosition(), robot.ticksPerInchSide * 48, 1),
                            Math.PI / 2, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

                        driveStep = "Hold";

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

                        holdTime = runtime.seconds();

                        firstTimeStone = false;
                    }

                    if(scissorControl(false, true, 3.5, 0)){
                        distanceHold = robot.distanceFront.getDistance(DistanceUnit.INCH);
                        driveStep = "To Foundation";
                        stoneStep = "Hold";
                        firstTimeStone = true;
                    }

                    break;

                case "Grab":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();

                        holdTime = runtime.seconds();

                        firstTimeStone = false;
                    }

                    if(scissorControl(false, false, 3, 0)){
                        distanceHold = robot.distanceFront.getDistance(DistanceUnit.INCH);
                        driveStep = "Backwards";
                        stoneStep = "Hold";
                        firstTimeStone = true;
                    }

                    break;

                case "Release":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();

                        holdTime = runtime.seconds();

                        firstTimeStone = false;
                    }

                    if(scissorControl(false, true, 2, 5)){

                        driveStep = "To Skybridge";

                        stoneStep = "Hold to Return";

                        holdTime = runtime.seconds();

                        robot.gripper.setPosition(robot.gripperClosed);

                        firstTimeStone = true;

                    }

                    break;

                case "Return":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();

                        holdTime = runtime.seconds();

                        firstTimeStone = false;
                    }

                    if(scissorControl(false, false, 3.5, 5)){

                        stoneStep = "Hold";

                        robot.gripper.setPosition(robot.gripperClosed);

                        firstTimeStone = true;

                    }

                    break;

                case "Hold to Return":

                    robot.scissor.setPower(0.0);
                    robot.scissor2.setPower(0.0);

                    robot.linear.setPower(0.0);

                    if(runtime.seconds() - holdTime > 1){
                        stoneStep = "Return";
                    }

                    break;

                case "Hold":

                    robot.scissor.setPower(0.0);
                    robot.scissor2.setPower(0.0);

                    robot.linear.setPower(0.0);

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

    private double heightToTicks(double desiredHeight){

        double heightToWidth = Math.sqrt(121 - Math.pow(desiredHeight / 5, 2));

        return heightToWidth * robot.scissorRevPerInch * robot.ticksPerScissorInch;

    }

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

    private boolean scissorControl(boolean grip, boolean moveUp, double scissorHeight, double linearPosition){

        if(grip){
            robot.gripper.setPosition(robot.gripperClosed);
        }else{
            robot.gripper.setPosition(robot.gripperOpen);
        }

        //robot.rotator.setPosition(rotatorPosition);

        //double currHeight = robot.distanceScissor.getDistance(DistanceUnit.INCH);

        /*if(scissorHeight - currHeight > 0 && Math.abs(scissorHeight - currHeight) > robot.distanceRange) {
            robot.scissor.setPower(-1);
            robot.scissor2.setPower(-1);
        }else if(scissorHeight - currHeight < 0 && Math.abs(scissorHeight - currHeight) > robot.distanceRange) {
            robot.scissor.setPower(1);
            robot.scissor2.setPower(1);
        }else{
            robot.scissor.setPower(0);
            robot.scissor2.setPower(0);
        }/*

        /*if(heightToTicks(scissorHeight) - robot.scissor.getCurrentPosition() > robot.distanceRange){
            robot.scissor.setPower(1);
            robot.scissor2.setPower(1);
        }else if(heightToTicks(scissorHeight) - robot.scissor.getCurrentPosition() < -robot.distanceRange){
            robot.scissor.setPower(-1);
            robot.scissor2.setPower(-1);
        }else{
            robot.scissor.setPower(0);
            robot.scissor2.setPower(0);
        }*/

        double scissorPower = 0;

        if(moveUp){
            scissorPower = -1;
        }else{
            scissorPower = 1;
        }

        if(runtime.seconds() - holdTime > scissorHeight){
            scissorPower = 0;
        }

        if(!lowerLimitState && scissorPower == 1){
            scissorPower = 0;
        }

        robot.scissor.setPower(scissorPower);
        robot.scissor2.setPower(scissorPower);

        linearPosition *= robot.ticksPerLinearInch;

        double linearPower = 0;

        /*if((linearPosition - robot.linear.getCurrentPosition()) <= -20 && backLimitState){
            linearPower = 0.5;
        }else if((linearPosition - robot.linear.getCurrentPosition()) >= 20 && frontLimitState){
            linearPower = -0.5;
        }else{
            linearPower = 0.0;
        }*/

        robot.linear.setPower(linearPower);

        //telemetry.addData("Curr Height", currHeight);
        telemetry.addData("Scissor Height", scissorHeight);

        return (scissorPower == 0) && (linearPower == 0);

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
