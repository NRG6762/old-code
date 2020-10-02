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

import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;

@Autonomous(name="REALLY DO NOT USE: Autonomous: Depot", group="Autonomous Over Testing")
//@Disabled
public class RoverAutonomousDepotTFodTesting extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private RoverHardware robot = new RoverHardware();

    //Telemetry Activators
    private boolean driveTelemetry = false;

    //Autonomous stage
    private String stage = "Move Left";
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

        //Update the telemetry
        telemetry.update();

        //Wait for the Pressing of the Start Button
        waitForStart();

        //Reset the game clock to zero in Start()
        runtime.reset();

        robot.tfod.activate();

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            switch (stage) {

                case "Move Left":
                    speed = driveCurve(2000, robot.drive4.getCurrentPosition(), .4);
                    robot.drive1.setPower(0);
                    robot.drive2.setPower(-speed);
                    robot.drive3.setPower(0);
                    robot.drive4.setPower(speed);
                    if (speed == 0) {
                        stopAndResetMotors(true, true);
                        robot.tfod.shutdown();
                        stage = "Exit";
                    }
                    break;

                case "Exit":

                    break;

            }

            if (robot.tfod != null) {
                List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int goldMineralX = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1) {
                        goldObj = true;
                        robot.tfod.shutdown();
                    } else {
                    }
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

            telemetry.addData("GoldPosition", goldPos);
            telemetry.addData("GoldMineralX", goldMineralX);
            telemetry.addData("GoldObject", goldObj);

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

};
