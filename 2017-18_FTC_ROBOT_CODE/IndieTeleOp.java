package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/*
        _    __   _   __   _   _           ___                      ___                _    _
 |\ |  |_)  /__  |_    /  |_    )    __     |   ._    _|  o   _      |    _   |   _   / \  |_)
 | \|  | \  \_|  |_)  /   |_)  /_          _|_  | |  (_|  |  (/_     |   (/_  |  (/_  \_/  |
 ---------------------------------------------------------------------------------------------
    Programmers: Aiden
                 Nole Sniekus
                 Chris Ferrotti

    School:     Northwest Regional 7

 */
@TeleOp(name="Indie Tele Op", group="@TeleOp")
public class IndieTeleOp extends LinearOpMode{

    //Declare OpMode members. Use the class created to define the robot's hardware
    private IndieHardware indie = new IndieHardware();
    private double startHeading;

    //Digital Sensor Booleans
    boolean ballLiftLow = false;

    //Telemetry activations
    private boolean joyPosTele          = false;
    private boolean joyPolarCoordTele   = false;
    private boolean wheelScalersTele    = false;
    private boolean wheelPositionTele   = false;
    private boolean liftPositionTele    = false;
    private boolean imuHeadingTele      = false;
    private boolean relicLiftTele       = false;

    double currentHeading;
    double speedDiv = 1;                    //Speed Divider for Slower Drive Actions

    double scissorRotationSpeed = 0.02;
    double currentScissorGoal = 0.0;

    double scissorTime = 0.0;
    double loopStart = 0.0;
    double currentTime = 0.0;

    @Override
    public void runOpMode() {

        //Initialize the hardware variables. The init() method of the hardware class does all the work here
        telemetry.addData("Init", "Initializing Hardware Map");
        telemetry.update();
        indie.init(hardwareMap);
        telemetry.addData("Init", "Initialized Hardware Map");
        telemetry.update();


        //startHeading = imuHeading();   //Get current Heading
        currentScissorGoal = 0.5;      //Set initial rotation (.5 = straight)

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        loopStart = getRuntime();

        //Initialize Gravity to get correct heading values back (-180 to 180)
        //indie.Imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Code to run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            currentTime = getRuntime() - loopStart;

            if (indie.driveBool) {

                //Create and set values to used variables for drive system
                double speed = 0, angle = 0, dchange = 0;
                double retValues[];

                //SpeedControl top off at 1/2 speed for 2x resolution
                if (gamepad1.left_bumper){
                    speedDiv = .5;
                }else{
                    speedDiv =1;
                }

                //CartesianToPolar method used for changing the cartesian values from joystick into polar values for drive
                retValues = cartesianToPolar(gamepad1.left_stick_y*speedDiv, gamepad1.left_stick_x*speedDiv, gamepad1.right_stick_x*speedDiv);

                //Set retValues array to shown values. retValues is used to return the variables used in an array.
                speed = retValues[0];
                angle = retValues[1];
                dchange = retValues[2];

                //PolarToWheelSpeed method used for powering wheels
                polarToWheelSpeed(speed, angle, dchange);
            }

            //For Testing IMU Heading
            /*if (imuHeadingTele){
                currentHeading = imuHeading();
            }*/

            //Instance of blockLiftControl method used for controlling the glyph lifter
            blockLiftControl(gamepad2.left_stick_y, gamepad2.right_stick_y);

            //Instance of scissorControl method used for controlling the relic lifter
            //scissorControl(gamepad2.dpad_up,gamepad2.dpad_down,gamepad2.dpad_left,gamepad2.dpad_right,gamepad2.left_bumper,gamepad2.right_bumper,gamepad2.right_trigger);

            //rgbLedsChangeState(indie.BallLiftLow.getState());
            if (!indie.BallLiftLow.getState()) {
                rgbLedsChangeState(true);
            } else {
                rgbLedsChangeState(false);
            }
            //CheckForWheelTelemetry
            if (wheelPositionTele) {
                telemetry.addData("Wheel Position1: ", indie.DrivePos1.getCurrentPosition());
                telemetry.addData("Wheel Position2: ", indie.DrivePos2.getCurrentPosition());
                telemetry.addData("Wheel Position3: ", indie.DrivePos3.getCurrentPosition());
                telemetry.addData("Wheel Position4: ", indie.DrivePos4.getCurrentPosition());
            }

           //CheckForLiftTelemetry
            if (liftPositionTele)  {
                telemetry.addData("Lift Position: ", indie.BlockLift.getCurrentPosition());
            }

            //CheckForIMUTelemetry
            if (imuHeadingTele) {
                telemetry.addData("IMU Heading: ", currentHeading);
                telemetry.addData("IMU Start Heading: ", startHeading);
            }

            if (relicLiftTele){
                telemetry.addData("Artifact Lift Pos: ",indie.ScissorArmLiftPwr.getCurrentPosition());
                telemetry.addData("Scissor Pos: ", indie.ScissorArmPwr.getCurrentPosition());
                telemetry.addData("Artifact Lift Sevro: ", indie.ScissorRotate.getPosition());
                telemetry.addData("Artifact Arm Servo: ", indie.RelicPivot.getPosition());
                telemetry.addData("Artifact Gripper Servo: ", indie.RelicGripper.getPosition());
            }

            //Update the Telemetry
            telemetry.update();
        }
    }

    private double[] cartesianToPolar(double y1, double x1, double x2) {

        //Reset retValues to 0 for later use
        double[] retValues = new double[]{0.0, 0.0, 0.0};
        double speed = 0.0, angle = 0.0, dchange = 0.0;

        //Create a deadzone for the main drive sticks
        gamepad1.setJoystickDeadzone((float) 0.05);
        gamepad2.setJoystickDeadzone((float) 0.05);

        //Change joypad values into useful polar values
        speed = Math.sqrt((y1 * y1) + (x1 * x1));
        angle = Math.atan2(x1, -y1);
        dchange = -x2;

        //Joypad input Telemetry
        if (joyPosTele) {
            telemetry.addData("X1: ", x1);
            telemetry.addData("Y1: ", y1);
            telemetry.addData("X2: ", x2);
        }

        //Polar values Telemetry
        if (joyPolarCoordTele) {
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Angle: ", angle);
            telemetry.addData("Rotational Change: ", dchange);
        }

        //Add polar values to retValues array
        retValues[0] = speed;
        retValues[1] = angle;
        retValues[2] = dchange;
        return retValues;
    }

    private void polarToWheelSpeed(double speed, double angle, double dchange){

        //Create Variables used only in this method
        double pos1, pos2, pos3, pos4, maxValue;

        //Define unscaled voltage multipliers
        pos1 = speed*Math.sin(angle+(Math.PI/4))+dchange;
        pos2 = speed*Math.cos(angle+(Math.PI/4))-dchange;
        pos3 = speed*Math.cos(angle+(Math.PI/4))+dchange;
        pos4 = speed*Math.sin(angle+(Math.PI/4))-dchange;

        //VOLTAGE MULTIPLIER SCALER

        //Set maxValue to pos1 absolute
        maxValue = Math.abs(pos1);

        //If pos2 absolute is greater than maxValue, then make maxValue equal to pos2 absolute
        if(Math.abs(pos2) > maxValue){maxValue = Math.abs(pos2);}

        //If pos3 absolute is greater than maxValue, then make maxValue equal to pos3 absolute
        if(Math.abs(pos3) > maxValue){maxValue = Math.abs(pos3);}

        //If pos4 absolute is greater than maxValue, then make maxValue equal to pos4 absolute
        if(Math.abs(pos4) > maxValue){maxValue = Math.abs(pos4);}

        //Check if need to scale -- if not set to 1 to nullify scale
        if (maxValue <= 1){ maxValue = 1;}

        //Power motors with scaled voltage multipliers
        indie.DrivePos1.setPower(pos1/maxValue);
        indie.DrivePos2.setPower(pos2/maxValue);
        indie.DrivePos3.setPower(pos3/maxValue);
        indie.DrivePos4.setPower(pos4/maxValue);

        //Scaled Voltage Multiplier Telemetry
        if (wheelScalersTele) {
            telemetry.addData("Wheel 1 W/ Scale: ",pos1/maxValue);
            telemetry.addData("Wheel 2 W/ Scale: ",pos2/maxValue);
            telemetry.addData("Wheel 3 W/ Scale: ",pos3/maxValue);
            telemetry.addData("Wheel 4 W/ Scale: ",pos4/maxValue);
        }
    }

    private void blockLiftControl(double leftStickY, double rightStickY){

        if(indie.blockLiftBool) {

            indie.GripperRight.setPosition(rightStickY);
            indie.GripperLeft.setPosition(rightStickY);

            if(indie.liftLimitBool){
                ballLiftLow = indie.BallLiftLow.getState();
                if(ballLiftLow){
                    indie.BlockLift.setPower(leftStickY);
                }else if(!ballLiftLow){
                    if(leftStickY <= 0.0){
                        indie.BlockLift.setPower(leftStickY);
                    }else{
                        indie.BlockLift.setPower(0);
                    }
                }else{
                    indie.BlockLift.setPower(leftStickY);
                }

                telemetry.addData("Block Lift: ",ballLiftLow);
            }
        }
    }

    private void scissorControl(boolean dPadUp, boolean dPadDown, boolean dPadLeft, boolean dPadRight, boolean leftBumper, boolean rightBumper, double rightTrigger){

        leftBumper = gamepad2.left_bumper;
        rightBumper = gamepad2.right_bumper;
        indie.ScissorRotate.setPosition(scissorRotationPosition(leftBumper, rightBumper));

        if(dPadLeft || dPadRight){
            if(dPadLeft){
                indie.ScissorArmPwr.setPower(.5);
            }else{
                indie.ScissorArmPwr.setPower(-.5);
            }
        }else{
            indie.ScissorArmPwr.setPower(0);
        }

        if(dPadUp || dPadDown){
            if(dPadUp){
                indie.ScissorArmLiftPwr.setPower(1);
            }else{
                indie.ScissorArmLiftPwr.setPower(-1);
            }
        }else{
            indie.ScissorArmLiftPwr.setPower(0);
        }

        if(rightTrigger >= .95) {
            indie.RelicGripper.setPosition(1);
        }
        else indie.RelicGripper.setPosition(0);

    }

    private double scissorRotationPosition(boolean leftBumper, boolean rightBumper){

        double goalPosition = 0.0;

        if(currentTime - scissorTime >= scissorRotationSpeed) {

            if (leftBumper) {
                goalPosition = goalPosition + scissorRotationSpeed;
            } else if (rightBumper) {
                goalPosition = goalPosition - scissorRotationSpeed;
            }



        }

        if (goalPosition >= 1){
            goalPosition = 1;
        }else if (goalPosition <=0){
            goalPosition = 0;
        }

        return goalPosition;

    }

    /*public double imuHeading() {
        double heading;
        try{
            //indie.Imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            Orientation angles = indie.Imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY);
            heading = indie.Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        }catch (Exception e){
            telemetry.addData("Init", "IMU blew up, disabled: " + e.toString());
            telemetry.update();
            heading = 0.0;
        }

        return heading;
    }*/

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void rgbLedsChangeState(boolean stateChange){
        indie.LeftRGBLED.setState(stateChange);
        indie.RightRGBLED.setState(stateChange);
    }

}
