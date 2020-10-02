package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

@Autonomous(name="DO NOT USE: Autonomous: Depot", group="Autonomous Testing")
//@Disabled
public class RoverAutonomousDepotTesting extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private RoverHardware robot = new RoverHardware();

    //Telemetry Activators
    private boolean driveTelemetry = false;

    //Autonomous stage
    private String stage = "Drop Down";
    private String aux = "Nothing";

    //Tensor Flow vairables
    private boolean goldObj = false;
    private String goldPos = "UNKNOWN";
    private int goldMineralX = -1;
    private int lightRun = 0;

    //Drive variables
    private boolean curveActive = true;
    private double speed = 0.0;

    //Lift variables
    private int liftTotalMove = 16000;

    //IMU Variables
    private Orientation angles;
    private double lastOrient;
    private double currOrient;
    private double startOrient;
    private double pureOrient;
    private double infOrient = 0;

    //Nole's Testing
    private int QuantORun = 0;

    @Override
    public void runOpMode() {

        //Initialize the Hardware Map
        robot.init(hardwareMap, true);

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Signify the Hardware Map has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Use Drive", robot.drive);
        telemetry.addData("Use Lift", robot.liftB);
        telemetry.addData("Use IMU", robot.BNO);
        telemetry.addData("Use Totem", robot.totemB);
        telemetry.addData("Use Rake", robot.rakeB);
        telemetry.addData("Use Vuforia", robot.vuforiaB);

        /*if (robot.vuforiaB) {
            //robot.vision.init();
            robot.vision.enable();
        }*/

        //Update the telemetry
        telemetry.update();

        //Wait for the Pressing of the Start Button
        waitForStart();

        //Reset the game clock to zero in Start()
        runtime.reset();

        /*if (robot.vuforiaB) {
            robot.vision.disable();
            robot.goldPosition = robot.vision.getTfLite().getLastKnownSampleOrder();
        }*/

        startOrient = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            currOrient = angles.firstAngle;

            if(Math.abs(currOrient - pureOrient) > 340){
                if(currOrient < pureOrient){
                    ++infOrient;
                }else if(currOrient > pureOrient){
                    --infOrient;
                }
            }

            currOrient = currOrient + (360 * infOrient);
            pureOrient = angles.firstAngle;


            switch (stage) {

                case "Drop Down":
                    robot.lift.setPower(-1.0);
                    if (robot.lift.getCurrentPosition() >= liftTotalMove) {
                        stopAndResetMotors(true, true);
                        robot.lift.setPower(0.0);
                        stage = "Da Twist";
                        //aux = "Lift Reset";
                        robot.light.setPower(1.0);
                    }
                    //stage = "Mineral: Move Out";
                    break;

                case "Da Twist":
                    if (currOrient > startOrient){
                        robot.drive1.setPower(-.4);
                        robot.drive2.setPower(-.4);
                        robot.drive3.setPower(-.4);
                        robot.drive4.setPower(-.4);
                    } else {
                        stopAndResetMotors(true, true);
                        stage = "MoveLeft";
                    }
                    break;

                case "MoveLeft":
                    speed = driveCurve(200, robot.drive2.getCurrentPosition(), .5);
                    robot.drive1.setPower(0);
                    robot.drive2.setPower(speed);
                    robot.drive3.setPower(0);
                    robot.drive4.setPower(-speed);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        stage = "Mineral: Move Out";
                    }
                    break;


                case "Mineral: Move Out":
                    speed = driveCurve(2500, robot.drive1.getCurrentPosition(), .8);
                    robot.drive1.setPower(speed);
                    robot.drive2.setPower(0);
                    robot.drive3.setPower(-speed);
                    robot.drive4.setPower(0);
                    if (speed == 0) {
                        if(goldObj){
                            stage = "Mineral: To Hit";
                        }else {
                            stage = "Mineral: Move Right";
                        }
                    }else if(speed > .75){
                        robot.tfod.activate();
                    }

                    break;

                case "Mineral: Move Right":
                    speed = driveCurve(1000, robot.drive2.getCurrentPosition(), .5);
                    robot.drive1.setPower(0);
                    robot.drive2.setPower(speed);
                    robot.drive3.setPower(0);
                    robot.drive4.setPower(-speed);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        if(goldObj){
                            stage = "Mineral: To Hit";
                        }else{
                            stage = "Mineral: Move Left to Left Mineral";
                        }
                    }
                    break;

                /*case "Mineral: Move Left-Center Mineral":
                    speed = driveCurve(1000, robot.drive4.getCurrentPosition(), .8);
                    robot.drive1.setPower(0);
                    robot.drive2.setPower(-speed);
                    robot.drive3.setPower(0);
                    robot.drive4.setPower(speed);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        //speed = 0.17;
                        stage = "Mineral: Move Left-Left Mineral";
                    }
                    break;*/

                case "Mineral: Move Left to Left Mineral":
                    speed = driveCurve(2000, robot.drive4.getCurrentPosition(), .8);
                    robot.drive1.setPower(0);
                    robot.drive2.setPower(-speed);
                    robot.drive3.setPower(0);
                    robot.drive4.setPower(speed);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        if (robot.tfod != null) {
                            robot.tfod.shutdown();
                        }
                        stage = "Mineral: To Hit";
                    }
                    break;


                case "Mineral: To Hit":
                    speed = driveCurve(500, robot.drive1.getCurrentPosition(), .9);
                    robot.drive1.setPower(speed);
                    robot.drive2.setPower(0);
                    robot.drive3.setPower(-speed);
                    robot.drive4.setPower(0);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        stage = "Mineral: Buffer 1";
                    }
                    break;

                case "Mineral: Buffer 1":
                    stage = "Mineral: From Hit";
                    break;


                case "Mineral: From Hit":
                    speed = driveCurve(600, robot.drive3.getCurrentPosition(), .9);
                    robot.drive1.setPower(-speed);
                    robot.drive2.setPower(0);
                    robot.drive3.setPower(speed);
                    robot.drive4.setPower(0);
                    QuantORun++;
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        stage = "Move Left";
                        robot.light.setPower(1);

                    }
                    break;

                case "Move Left":
                    speed = driveCurve(2000, robot.drive4.getCurrentPosition(), .8);
                    robot.drive1.setPower(0);
                    robot.drive2.setPower(-speed);
                    robot.drive3.setPower(0);
                    robot.drive4.setPower(speed);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        stage = "To Turn";
                    }
                    break;

                case "To Turn":
                    speed = driveCurve(1400, robot.drive1.getCurrentPosition(), .8);
                    robot.drive1.setPower(speed);
                    robot.drive2.setPower(0);
                    robot.drive3.setPower(-speed);
                    robot.drive4.setPower(0);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        stage = "Turn";
                    }
                    break;

                case "Turn":
                    if (currOrient > startOrient - 40) {
                        robot.drive1.setPower(-.2);
                        robot.drive2.setPower(-.2);
                        robot.drive3.setPower(-.2);
                        robot.drive4.setPower(-.2);
                    } else {
                        stopAndResetMotors(true, true);
                        stage = "To Depot";
                        lastOrient = currOrient;
                    }
                    break;

                case "To Depot":
                    speed = driveCurve(2600, robot.drive1.getCurrentPosition(), 1);
                    robot.drive1.setPower(speed);
                    robot.drive2.setPower(0);
                    robot.drive3.setPower(-speed);
                    robot.drive4.setPower(0);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        stage = "Drop It";
                    }
                    break;

                case "Drop It":
                    robot.totem.setPosition(0.7);
                    stage = "Return to Crater";
                    break;

                case "Return to Crater":
                    speed = driveCurve(5000, robot.drive3.getCurrentPosition(), .9);
                    robot.drive1.setPower(-speed);
                    robot.drive2.setPower(0);
                    robot.drive3.setPower(speed);
                    robot.drive4.setPower(0);
                    if (speed == 0) {
                        stopAndResetMotors(true,true);
                        stage = "Spin";
                    }
                    break;

                case "Spin":
                    robot.totem.setPosition(0.2);
                    if (currOrient < lastOrient - 180) {
                        robot.drive1.setPower(-.4);
                        robot.drive2.setPower(-.4);
                        robot.drive3.setPower(-.4);
                        robot.drive4.setPower(-.4);
                        QuantORun++;
                    } else {
                        stopAndResetMotors(true,true);
                        stage = "Exit";
                    }
                    break;

                case "Exit":
                    robot.totem.setPosition(0.7);
                    break;



            }

            switch(aux){
                    case "Lift Reset":
                        robot.lift.setPower(0.5);
                        if(robot.lift.getCurrentPosition() <= -liftTotalMove + 100){
                            robot.lift.setPower(0.0);
                            aux = "Nothing";
                        }
                        break;

            }

            if (robot.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = null;
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if(updatedRecognitions.size() == 0){
                    }else if(updatedRecognitions.size() == 1) {
                        goldMineralX = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            }
                        }
                        telemetry.addData("GoldMineralX", goldMineralX);
                        if (goldMineralX != -1){
                            goldObj = true;
                            robot.tfod.shutdown();
                        }else{
                            goldObj = false;
                        }
                        telemetry.addData("GoldObj", goldObj);
                    }else{
                        goldObj = false;
                    }
                }else{
                    goldObj = false;
                }

                if(goldObj){
                    switch (stage){
                        case "Mineral: Move Out":
                            goldPos = "CENTER";
                            break;

                        case "Mineral: Move Right":
                            goldPos = "RIGHT";
                            break;

                        case "Mineral: Move Left to Left Mineral":
                            goldPos = "LEFT";
                            break;
                    }
                }else{
                    goldPos = "UNKNOWN";
                    //robot.light.setPower(1);
                }

            }else {
                if (goldObj && !goldPos.equals("UNKNOWN")) {
                    ++lightRun;
                    if (lightRun < 50) {
                        //robot.light.setPower(0);
                    } else if ((lightRun >= 50) && (lightRun < 100)) {
                        //robot.light.setPower(1);
                    } else if ((lightRun >= 100) && (lightRun < 150)) {
                        //robot.light.setPower(0);
                    } else if ((lightRun >= 150) && (lightRun < 200)) {
                        //robot.light.setPower(1);
                    } else {
                        //robot.light.setPower(0);
                    }
                }else{
                    robot.light.setPower(0);
                }
            }

            if (driveTelemetry) {
                //Drive Speed Telemetry
                telemetry.addData("Drive 1 Speed", robot.drive1.getPower());
                telemetry.addData("Drive 2 Speed", robot.drive2.getPower());
                telemetry.addData("Drive 3 Speed", robot.drive3.getPower());
                telemetry.addData("Drive 4 Speed", robot.drive4.getPower());
                telemetry.addData("Drive 1 Posit", robot.drive1.getCurrentPosition());
                telemetry.addData("Drive 2 Posit", robot.drive2.getCurrentPosition());
                telemetry.addData("Drive 3 Posit", robot.drive3.getCurrentPosition());
                telemetry.addData("Drive 4 Posit", robot.drive4.getCurrentPosition());
                if(robot.liftB) {
                    telemetry.addData("Lift Power", robot.lift.getPower());
                    telemetry.addData("Lift Pos", robot.lift.getCurrentPosition());
                }
            }

            telemetry.addLine(stage);

            telemetry.addData("StartOrient", startOrient);
            telemetry.addData("CurrOrient", currOrient);

            //telemetry.addData("Gold Mineral Position", robot.goldPosition);

            //Nole's Debugging
            telemetry.addData("Quantitiy of Runs", QuantORun);

            //Update the Telemetry
            telemetry.update();
        }
    }

    private void stopAndResetMotors(boolean power, boolean encoders) {
        if (power) {
            robot.drive1.setPower(0);
            robot.drive2.setPower(0);
            robot.drive3.setPower(0);
            robot.drive4.setPower(0);
        }
        if (encoders) {
            robot.drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.drive3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.drive4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.drive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.drive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.drive3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.drive4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private double driveCurve(double targetDistance, double currentDistance, double maxPower) {
        double outputPower = 0.0;
        if (currentDistance < 0) {
            currentDistance = -currentDistance;
        }
        currentDistance = currentDistance + 1;
        if (curveActive) {
            if (targetDistance - currentDistance > targetDistance / 2) {
                outputPower = maxPower * Math.sqrt(2 * currentDistance / targetDistance);
            } else if (targetDistance - currentDistance == targetDistance / 2) {
                outputPower = maxPower;
            } else if ((targetDistance - currentDistance < targetDistance / 2) && (targetDistance - currentDistance > 0)) {
                if(targetDistance <= 500) {
                    outputPower = -8 * maxPower / Math.pow(targetDistance, 3) * Math.pow(currentDistance - targetDistance, 3);
                }else{
                    outputPower = -4 * maxPower / Math.pow(targetDistance, 2) * Math.pow(currentDistance - (targetDistance / 2), 2) + maxPower;
                }
            } else {
                outputPower = 0;
            }
        } else {
            if (currentDistance >= 0 && currentDistance <= targetDistance) {
                outputPower = maxPower;
            } else {
                outputPower = 0.0;
            }
        }
        if ((outputPower < 0.17) && (outputPower != 0)) {
            outputPower = 0.17;
        }
        return outputPower;
    }

    /*private double rampUp(double looptime,int Di) {
        double A = Di/Math.sqrt(MaxRotBlah);
        return A * Math.sqrt(looptime);
    }

    private double rampDown(double Diff) {
        double H = MinDegreeRampDown;
        double A = -1 / Math.pow(H,3);
        return  Math.pow(Diff,3) * A;
    }

    public double diff(double value1, double value2) {
        double total = 0.0;
        if (value1 > value2) {
            total = ((value1 - value2));
        } else {
            total = ((value2 - value1));
        }
        return total;
    }*/

    public double correction(double oldHeading, double newHeading) {
        double corrSpin = 0.0;
        if (oldHeading - newHeading > 180) {
            if (oldHeading > newHeading) {
                corrSpin = (newHeading + 360) - oldHeading;
            } else if (oldHeading < newHeading) {
                corrSpin = (oldHeading + 360) - newHeading;
            } else if (oldHeading == newHeading) {
                corrSpin = 0.0;
            }
        }
        return corrSpin;
    }

    public double targetHeading(double startHeading, double headingDifference) {
        double difference = 0.0;
        difference = startHeading + headingDifference;
        if (difference > 180) {
            difference = difference - 360;
        } else if (difference < -180) {
            difference = difference + 360;
        }
        return difference;
    }

    public double orientDifference(double currOrient, double lastOrient) {
        double diff = 0.0;

        if (currOrient < 0) {
            currOrient = currOrient + 360;
        }

        return diff;
    }

};
