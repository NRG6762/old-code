package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/*
 * NRG6762
 * Northwestern Regional 7 Gearheads
 * 2019-2020 Season - Skystone
 * Autonomous Class - Base
 * Written by Aiden Maraia
 * Version: 1/21/2020
 * Feel free to make any changes and use at your disposal.
 */
@Autonomous(name="Builder Autonomous - Go Left", group="Competition")
//@Disabled
public class BuilderAutonomousGoLeft extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private BuilderHardware robot = new BuilderHardware(true);

    //Declare the use of mecanum telemetry
    private boolean mecanumTelemetry    = true;
    private boolean firstTime = true;

    public String driveStep = "Go Right";

    private Orientation angles;
    private double lastOrient;
    private double currOrient;
    private double startOrient;
    private double pureOrient;
    private long rotationCount = 0;

    private double ticksHold    = 0;

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

            switch (driveStep) {

                case "Go Right":

                    if(firstTime){

                        ticksHold = robot.rightFront.getCurrentPosition();

                        firstTime = false;

                    }

                    if(newDriveCommand(motorCurve(ticksHold, robot.rightFront.getCurrentPosition(), robot.ticksPerInchSide * 48, 1),
                            -Math.PI / 2, -correction(startOrient, currOrient))){

                        stopAndResetMotors();

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

            /*switch(senseStep){

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

            }*/

            telemetry.addData("Drive Step", driveStep);
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

        scissorHeight += robot.gripperDown;

        double currHeight = robot.distanceScissor.getDistance(DistanceUnit.INCH);

        if(scissorHeight - currHeight > 0 && Math.abs(scissorHeight - currHeight) > 1) {
            robot.scissor.setPower(-1);
        }else if(scissorHeight - currHeight < 0 && Math.abs(scissorHeight - currHeight) > 1) {
            robot.scissor.setPower(1);
        }else{
            robot.scissor.setPower(0);
        }

        if(!robot.limitLower.getState() || scissorHeight == 0 || Math.abs(currHeight - scissorHeight) < 1){
            robot.scissor.setPower(0);
        }

        if(!robot.limitLower.getState() || scissorHeight == 0){
            return true;
        }

        //robot.linear.setPower(motorCurve(holdTicksLinear, robot.linear.getCurrentPosition(), linearPosition * robot.ticksPerLinearInch, 1));

        return Math.abs(currHeight - scissorHeight) < 1 || !robot.limitLower.getState() /*&& Math.abs(robot.linear.getCurrentPosition() - (linearPosition * robot.ticksPerLinearInch)) <= 20*/;

    }

    public double correction(double oldHeading, double newHeading){

        double corrSpin = 0.0;

        if(oldHeading - newHeading > Math.PI) {
            if (oldHeading > newHeading) {
                corrSpin = (newHeading + 2 * Math.PI) - oldHeading;
            } else if (oldHeading < newHeading) {
                corrSpin = (oldHeading + 2 * Math.PI) - newHeading;
            } else if (oldHeading == newHeading) {
                corrSpin = 0.0;
            }
        }

        return  corrSpin;
    }

}
