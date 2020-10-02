package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/*
 * NRG6762
 * Northwestern Regional 7 Gearheads
 * 2019-2020 Season - Skystone
 * Autonomous Class - Base
 * Written by Aiden Maraia
 * Version: 1/21/2020
 * Feel free to make any changes and use at your disposal.
 */
@Autonomous(name="Just Flip", group="Competition")
//@Disabled
public class BuilderAutonomousDummyRight extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private BuilderHardware robot = new BuilderHardware(true);

    //Declare the use of mecanum telemetry
    private boolean mecanumTelemetry    = false;
    /*private boolean firstTime = true;
    private boolean firstTimeStone = true;
    private boolean skystoneSeen = false;
    private boolean dropedStone = false;
    private boolean wallHit = false;
    private boolean tapeSeen = false;

    public String driveStep = "To Stones";
    public String stoneStep = "Start";
    public String senseStep = "Init Vision";*/

    private Orientation angles;
    private double lastOrient;
    private double currOrient;
    private double startOrient;
    private double pureOrient;
    private long rotationCount = 0;

    private double distanceAHold = 0;
    private double distanceBHold = 0;
    private double distanceExtHold = 0;

    private double holdTicksScissor = 0;
    private double holdTicksLinear = 0;

    /* Code to be run while the robot is running (from pressing play to stop) */
    @Override
    public void runOpMode() {

        //Initialize the Hardware Map
        robot.init(hardwareMap);

        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 500);

        //Signify the Hardware Map has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Use Drive", robot.driveMotorsTrue);
        telemetry.addData("Use Scissor", robot.scissorDriveTrue);

        //Update the telemetry
        telemetry.update();

        /*VuforiaTrackables targetsSkyStone = this.robot.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, robot.stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if(robot.CAMERA_CHOICE == BACK) {
            robot.phoneYRotate = -90;
        } else {
            robot.phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if(robot.PHONE_IS_PORTRAIT) {
            robot.phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * robot.mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * robot.mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, robot.phoneYRotate, robot.phoneZRotate, robot.phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        /*for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, robot.parameters.cameraDirection);
        }*/

        //Wait for the Pressing of the Start Button
        waitForStart();

        //Reset the game clock to zero in Start()
        runtime.reset();

        //startOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*pureOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - startOrient;

            currOrient = pureOrient;

            if((pureOrient - lastOrient) > 5){
                rotationCount++;
            }else if((pureOrient - lastOrient) < -5){
                rotationCount--;
            }

            currOrient = pureOrient + rotationCount * 2 * Math.PI;*/

            /*switch (driveStep) {

                case "To Stones":

                    if(firstTime){
                        senseStep = "Init Vision";

                        distanceAHold = robot.distanceA.getDistance(DistanceUnit.INCH);
                        distanceBHold = robot.distanceB.getDistance(DistanceUnit.INCH);

                        distanceExtHold = distanceAHold;

                        if(distanceBHold > distanceExtHold){
                            distanceExtHold = distanceBHold;
                        }

                        firstTime = false;
                    }

                    robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * (distanceExtHold - 0.01), 0, startOrient, currOrient, startOrient)[0]);
                    robot.rightFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * (distanceExtHold - 0.01), 0, startOrient, currOrient, startOrient)[1]);
                    robot.leftBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * (distanceExtHold - 0.01), 0, startOrient, currOrient, startOrient)[2]);
                    robot.rightBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * (distanceExtHold - 0.01), 0, startOrient, currOrient, startOrient)[3]);

                    if(robot.distanceA.getDistance(DistanceUnit.INCH) <= 0.05 || robot.distanceB.getDistance(DistanceUnit.INCH) <= 0.05){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        driveStep = "Crab Along Stones";
                        firstTime = true;
                    }

                    break;

                case "Crab Along Stones":

                    if(firstTime){

                        firstTime = false;
                    }

                    robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 24, Math.PI / 2, startOrient, currOrient, startOrient)[0]);
                    robot.rightFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 24, Math.PI / 2, startOrient, currOrient, startOrient)[1]);
                    robot.leftBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 24, Math.PI / 2, startOrient, currOrient, startOrient)[2]);
                    robot.rightBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 24, Math.PI / 2, startOrient, currOrient, startOrient)[3]);

                    if(skystoneSeen){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        driveStep = "Grab Stone";
                        firstTime = true;
                    }

                    break;

                case "Grab Stone":

                    stoneStep = "Down & Open";

                    break;

                case "To Foundation":

                    if(firstTime){
                        firstTime = false;
                    }

                    robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, -Math.PI / 2, startOrient, currOrient, startOrient)[0]);
                    robot.rightFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, -Math.PI / 2, startOrient, currOrient, startOrient)[1]);
                    robot.leftBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, -Math.PI / 2, startOrient, currOrient, startOrient)[2]);
                    robot.rightBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, -Math.PI / 2, startOrient, currOrient, startOrient)[3]);

                    if(skystoneSeen){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        stoneStep = "Down To Drop";
                        firstTime = true;
                    }

                    break;

                case "Align to Foundation":

                    if(firstTime){

                        distanceAHold = robot.distanceA.getDistance(DistanceUnit.INCH);
                        distanceBHold = robot.distanceB.getDistance(DistanceUnit.INCH);

                        distanceExtHold = distanceAHold;

                        if(distanceBHold > distanceExtHold){
                            distanceExtHold = distanceBHold;
                        }

                        firstTime = false;

                    }

                    if(robot.distanceA.getDistance(DistanceUnit.INCH) < 0.01 && robot.distanceB.getDistance(DistanceUnit.INCH) < 0.01){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        driveStep = "Drop Stone";
                        firstTime = true;
                    }else{
                        robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * distanceExtHold, 0, startOrient, currOrient, startOrient)[0]);
                        robot.rightFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * distanceExtHold, 0, startOrient, currOrient, startOrient)[1]);
                        robot.leftBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * distanceExtHold, 0, startOrient, currOrient, startOrient)[2]);
                        robot.rightBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * distanceExtHold, 0, startOrient, currOrient, startOrient)[3]);
                    }

                    break;

                case "Drop Stone":

                    if(runtime.seconds() >= 15){
                        driveStep = "Crab Return";
                        firstTime = true;
                    }else{
                        driveStep = "Grab Foundation";
                        firstTime = true;
                    }

                    break;

                case "Crab Return":

                    if(firstTime){

                        firstTime = false;
                    }

                    robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, Math.PI / 2, startOrient, currOrient, startOrient)[0]);
                    robot.rightFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, Math.PI / 2, startOrient, currOrient, startOrient)[1]);
                    robot.leftBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, Math.PI / 2, startOrient, currOrient, startOrient)[2]);
                    robot.rightBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 96, Math.PI / 2, startOrient, currOrient, startOrient)[3]);

                    if((robot.leftFront.getPower() == 0) || (robot.rightFront.getPower() == 0) || (robot.leftBack.getPower() == 0) || (robot.rightBack.getPower() == 0)){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        driveStep = "Grab Stone";
                        firstTime = true;
                    }
                    break;

                case "Grab Foundation":

                    robot.foundationA.setPosition(robot.foundationClosed);
                    robot.foundationB.setPosition(robot.foundationClosed);

                    driveStep = "Move Foundation";

                    break;

                case "Move Foundation":

                    if(firstTime){

                        firstTime = false;
                    }

                    robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 36, Math.PI * 2, startOrient, currOrient, startOrient)[0]);
                    robot.rightFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 36, Math.PI * 2, startOrient, currOrient, startOrient)[1]);
                    robot.leftBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 36, Math.PI * 2, startOrient, currOrient, startOrient)[2]);
                    robot.rightBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 36, Math.PI * 2, startOrient, currOrient, startOrient)[3]);

                    if(wallHit || (robot.leftFront.getPower() == 0) || (robot.rightFront.getPower() == 0) || (robot.leftBack.getPower() == 0) || (robot.rightBack.getPower() == 0)){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        driveStep = "Release Foundation";
                        firstTime = true;
                    }

                    break;

                case "Release Foundation":

                    robot.foundationA.setPosition(robot.foundationOpen);
                    robot.foundationB.setPosition(robot.foundationOpen);

                    driveStep = "To Skybridge";

                    break;

                case "To Skybridge":

                    if(firstTime){
                        senseStep = "Tape Detector";
                        firstTime = false;
                    }

                    robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), robot.ticksPerDriveInch * 48, Math.PI / 2, startOrient, currOrient, startOrient)[0]);
                    robot.rightFront.setPower(driveCommand(0, robot.rightFront.getCurrentPosition(), robot.ticksPerDriveInch * 48, Math.PI / 2, startOrient, currOrient, startOrient)[1]);
                    robot.leftBack.setPower(driveCommand(0, robot.leftBack.getCurrentPosition(), robot.ticksPerDriveInch * 48, Math.PI / 2, startOrient, currOrient, startOrient)[2]);
                    robot.rightBack.setPower(driveCommand(0, robot.rightBack.getCurrentPosition(), robot.ticksPerDriveInch * 48, Math.PI / 2, startOrient, currOrient, startOrient)[3]);

                    if(tapeSeen){
                        robot.leftFront.setPower(0);
                        robot.rightFront.setPower(0);
                        robot.leftBack.setPower(0);
                        robot.rightBack.setPower(0);
                        driveStep = "Park";
                        firstTime = true;
                    }

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

                    if(scissorControl(false, robot.rotatorMidPosition, 5, 0)){
                        stoneStep = "Hold";

                    }

                    break;

                case "Down & Open":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(false, robot.rotatorMidPosition, 0, 0)){
                        stoneStep = "Close";
                        firstTimeStone = true;
                    }

                    break;

                case "Close":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(true, robot.rotatorMidPosition, 0, 0)){
                        stoneStep = "Uppy";
                        driveStep = "To Foundation";
                        firstTimeStone = true;
                    }

                    break;

                case "Uppy":

                    if(firstTimeStone){
                        holdTicksLinear = robot.linear.getCurrentPosition();
                        holdTicksScissor = robot.scissor.getCurrentPosition();
                        firstTimeStone = false;
                    }

                    if(scissorControl(true, robot.rotatorMidPosition, 1.5, 0)){
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

                    if(scissorControl(false, robot.rotatorMidPosition, 1, 0)){
                        stoneStep = "Hold";
                        driveStep = "Drop Stone";
                        firstTimeStone = true;
                    }

                    break;

                case "Hold":

                    robot.scissor.setPower(0.0);

                    robot.linear.setPower(0.0);

                    break;
            }

            switch(senseStep){

                case "Init Vision":
                    targetsSkyStone.activate();
                    senseStep = "Find Skystone";
                    break;

                case "Find Skystone":

                    skystoneSeen = false;

                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            skystoneSeen = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                robot.lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }

                    if(skystoneSeen){
                        senseStep = "End Vision";
                    }

                    break;

                case "End Vision":
                    targetsSkyStone.deactivate();
                    break;

                case "Tape Detector":

                    break;

            }*/

            /*robot.leftFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), 100, Math.PI / 2, startOrient, currOrient, startOrient)[0]);
            robot.rightFront.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), 100, Math.PI / 2, startOrient, currOrient, startOrient)[1]);
            robot.leftBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), 100, Math.PI / 2, startOrient, currOrient, startOrient)[2]);
            robot.rightBack.setPower(driveCommand(0, robot.leftFront.getCurrentPosition(), 100, Math.PI / 2, startOrient, currOrient, startOrient)[3]);*/

            robot.flipper.setPosition(robot.flipperOpen);

            //lastOrient = pureOrient;

            //Update the Telemetry
            telemetry.update();

        }

    }

    private double[] driveCommand(double startTicks, double currTicks, double goalTicks, double polarAngle, double startOrient, double currOrient, double goalOrient){

        double finalReturns[] = new double[4];

        double polarSpeed = motorCurve(startTicks, currTicks, goalTicks);

        double polarSpeen = 0;

        if(startOrient == goalOrient){

            polarSpeen = 0;

        }else{

            polarSpeen = motorCurve(startOrient, currOrient, goalOrient) / 3;

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

        /* Mecanum Telemetry for troubleshooting */
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
            telemetry.addData("Scaling Value", maxValue);*/

            //Show the calculated and scaled values
            telemetry.addData("Left Front Actual Value", finalReturns[0]);
            telemetry.addData("Right Front Actual Value", finalReturns[1]);
            telemetry.addData("Left Back Actual Value", finalReturns[2]);
            telemetry.addData("Right Back Actual Value", finalReturns[3]);

            //Update telemetry
            telemetry.update();

        }

        return finalReturns;

    }

    private double heightToTicks(double desiredHeight){

        double heightToWidth = Math.sqrt(121 - Math.pow(desiredHeight / 5, 2));

        return heightToWidth * robot.scissorRevPerInch * robot.ticksPerScissorInch;

    }

    private double motorCurve(double startTicks, double currTicks, double goalTicks){

        double curve;

        double rangeTicks = goalTicks - startTicks;

        double shiftedCurrTicks = currTicks - startTicks;

        if((rangeTicks - shiftedCurrTicks) / rangeTicks < .4){

            curve = Math.pow(shiftedCurrTicks / (.4 * rangeTicks), 1 / 3);

            if(curve < .2){
                curve = .2;
            }

        }else{

            curve = -Math.pow((shiftedCurrTicks - rangeTicks) / (.6 * rangeTicks), 3);

        }

        return curve;

    }

    private boolean scissorControl(boolean grip, double rotatorPosition, double scissorHeight, double linearPosition){

        if(grip){
            robot.gripper.setPosition(robot.gripperClosed);
        }else{
            robot.gripper.setPosition(robot.gripperOpen);
        }

        //robot.rotator.setPosition(rotatorPosition);

        robot.scissor.setPower(motorCurve(holdTicksScissor, robot.scissor.getCurrentPosition(), heightToTicks(scissorHeight)));

        robot.linear.setPower(motorCurve(holdTicksLinear, robot.linear.getCurrentPosition(), linearPosition * robot.ticksPerLinearInch));

        if(Math.abs(robot.scissor.getCurrentPosition() - heightToTicks(scissorHeight)) <= 20 && Math.abs(robot.linear.getCurrentPosition() - (linearPosition * robot.ticksPerLinearInch)) <= 20){
            return true;
        }

        return false;

    }

}
