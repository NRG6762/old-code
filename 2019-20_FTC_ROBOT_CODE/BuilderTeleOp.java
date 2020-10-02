package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * NRG6762
 * Northwestern Regional 7 Gearheads
 * 2019-2020 Season - Skystone
 * Teleop Class
 * Written by Aiden Maraia
 * Version: 1/21/2020
 * Feel free to make any changes and use at your disposal.
 */
@TeleOp(name="Builder Tele-Op", group="Competition")
//@Disabled
public class BuilderTeleOp extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private BuilderHardware robot = new BuilderHardware(false);

    //Declare a joystick deadzone
    private float deadzone = 0.025f;

    //Declare the use of telemetry
    private boolean mecanumTelemetry    = true;
    private boolean scissorTelemetry    = true;
    private boolean gripperTelemetry    = true;
    private boolean foundationTelemetry = true;

    private Boolean gripperDrive        = false;

    private Boolean scissorManual       = true;

    private int level = 0;
    private boolean levelBoolean = true;
    private boolean rotatorBoolean = true;
    private boolean foundationBoolean = true;
    private double holdTicksA = 0;
    private double holdTicksB = 0;
    private double rotatorCurrPosition = robot.rotatorMidPosition;

    /* Code to be run while the robot is running (from pressing play to stop) */
    @Override
    public void runOpMode() {

        //Initialize the Hardware Map
        robot.init(hardwareMap);

        //Signify the Hardware Map has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Use Drive", robot.driveMotorsTrue);
        telemetry.addData("Use Scissor", robot.scissorDriveTrue);

        //Update the telemetry
        telemetry.update();

        //Wait for the Pressing of the Start Button
        waitForStart();

        //Reset the game clock to zero in Start()
        runtime.reset();

        //Set the gripper position to open
        //robot.gripper.setPosition(robot.gripperOpen);


        //DefineLimitSwicthPlaceholders
        //set to false in case we are not using limit switches
        boolean upperLimitState = false;
        boolean lowerLimitState = false;
        boolean frontLimitState = false;
        boolean backLimitState = false;

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Create a deadzone for the joysticks
            gamepad1.setJoystickDeadzone(deadzone);
            gamepad2.setJoystickDeadzone(deadzone);

            //Turn joystick input into drive motor movement
            if(robot.driveMotorsTrue){

                //Change joystick input into speed, angle, and spin
                double polarSpeed = Math.sqrt((gamepad1.left_stick_y * gamepad1.left_stick_y) + (gamepad1.left_stick_x * gamepad1.left_stick_x));
                double polarAngle = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double polarSpeen = -gamepad1.right_stick_x / 2;

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

                //Send the scaled power levels to drive motors
                robot.leftFront.setPower(leftFrontValue / maxValue);
                robot.rightFront.setPower(rightFrontValue / maxValue);
                robot.leftBack.setPower(leftBackValue / maxValue);
                robot.rightBack.setPower(rightBackValue / maxValue);

                /* Mecanum Telemetry for troubleshooting */
                if(mecanumTelemetry) {

                    //Show the calculated vector values
                    telemetry.addData("Polar Speed", polarSpeed);
                    telemetry.addData("Polar Angle", polarAngle);
                    telemetry.addData("Polar Speen", polarSpeen);

                    //Show the calculated, unscaled values
                    telemetry.addData("Left Front Value", leftFrontValue);
                    telemetry.addData("Right Front Value", rightFrontValue);
                    telemetry.addData("Left Back Value", leftBackValue);
                    telemetry.addData("Right Back Value", rightBackValue);

                    //Show the scaling values
                    telemetry.addData("Scaling Value", maxValue);

                    //Show the encoder values
                    telemetry.addData("Left Front Encoder", robot.leftFront.getCurrentPosition());
                    telemetry.addData("Right Front Encoder", robot.rightFront.getCurrentPosition());
                    telemetry.addData("Left Back Encoder", robot.leftBack.getCurrentPosition());
                    telemetry.addData("Right Back Encoder", robot.rightBack.getCurrentPosition());

                }
            }

            if(robot.scissorDriveTrue) {

                /*if(levelBoolean && gamepad2.dpad_up){
                    level++;
                    levelBoolean = false;
                }else if(levelBoolean && gamepad2.dpad_down){
                    level--;
                    levelBoolean = false;
                }else{
                    levelBoolean = true;
                }

                if(robot.scissor.getCurrentPosition() == heightToTicks(4 * level)){
                    holdTicksA = robot.scissor.getCurrentPosition();
                }else {
                    robot.scissor.setPower(motorCurve(holdTicksA, robot.scissor.getCurrentPosition(), heightToTicks(4 * level)));
                }*/

                double liftPower = gamepad2.left_stick_y;


                if (robot.limitTrue) {
                    upperLimitState = robot.limitUpper.getState();
                    lowerLimitState = robot.limitLower.getState();
                    frontLimitState = robot.limitFront.getState();
                    backLimitState = robot.limitBack.getState();
                }

                if ((liftPower < 0) && (!lowerLimitState)){
                    liftPower = 0;
                telemetry.addLine("Lift Down All The Way! Stop It!");

                }

                if( (liftPower > 0) && (!upperLimitState)){
                    liftPower = 0;
                    telemetry.addLine("Lift Up All The Way! Stop It!");
                }

                robot.scissor.setPower(liftPower);
                robot.scissor2.setPower(liftPower);


                double linearPower = gamepad2.right_stick_y/2;

                if((linearPower < 0) && (backLimitState)) {
                    linearPower = 0;
                    telemetry.addLine("Slide Back All The Way! Stop It!");
                }

                if( (linearPower > 0) && (frontLimitState )){
                    linearPower = 0;
                    telemetry.addLine("Slide Forward All The Way! Stop It!");
                }

                robot.linear.setPower(linearPower);

                if(scissorTelemetry){
                    telemetry.addData("Scissor Encoder", robot.scissor.getCurrentPosition());
                    telemetry.addData("Linear Encoder", robot.linear.getCurrentPosition());
                }

            }

            if(robot.gripperMechTrue) {

                if (gamepad2.a) {
                    robot.gripper.setPosition(robot.gripperClosed);
                } else if (gamepad2.b) {
                    robot.gripper.setPosition(robot.gripperOpen);
                }

                if (gamepad2.x) {
                    robot.flipper.setPosition(robot.flipperClosed);
                } else if (gamepad2.y) {
                    robot.flipper.setPosition(robot.flipperOpen);
                }

                if (rotatorBoolean && gamepad2.right_bumper) {
                    rotatorCurrPosition += .05;
                    rotatorBoolean = false;
                } else if (rotatorBoolean && gamepad2.left_bumper) {
                    rotatorCurrPosition -= .05;
                    rotatorBoolean = false;
                } else {
                    rotatorBoolean = true;
                }

                robot.rotator.setPosition(rotatorCurrPosition);

                if(gripperTelemetry) {
                    telemetry.addData("Gripper Position", robot.gripper.getPosition());
                    telemetry.addData("Flipper Position", robot.flipper.getPosition());
                    telemetry.addData("Rotator Position", robot.rotator.getPosition());
                    telemetry.addData("Lift Distance", robot.distanceScissor.getDistance(DistanceUnit.INCH));
                }

            }

            if(robot.foundationTrue){

                if(gamepad2.dpad_down){
                    robot.foundationA.setPosition(robot.foundationClosed);
                    robot.foundationB.setPosition(robot.foundationClosed);
                }else if(gamepad2.dpad_up){
                    robot.foundationA.setPosition(robot.foundationOpen);
                    robot.foundationB.setPosition(robot.foundationOpen);
                }

                if(foundationTelemetry){
                    telemetry.addData("Foundation Position", robot.foundationA.getPosition());
                }

            }

            //Update the Telemetry
            telemetry.update();

        }

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

}
