package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name="Rover Tele-Op", group="Linear Opmode")
//@Disabled
public class RoverTeleOp extends LinearOpMode {

    //Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private RoverHardware robot = new RoverHardware();

    //Telemetry Activators
    private boolean driveTelemetry = true;

    //Drive Variables
    private double[] returnedDriveSpeed = new double[4];
    private double   joystickAngle;
    private double   totemPos = 0.0;

    //IMU Variables
    private Orientation angles;
    private Acceleration gravity;

    @Override
    public void runOpMode() {

        //Initialize the Hardware Map
        robot.init(hardwareMap,false);

        //Signify the Hardware Map has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Use Drive", robot.drive);
        telemetry.addData("Use Lift", robot.liftB);
        telemetry.addData("Use IMU", robot.BNO);
        telemetry.addData("Use Totem", robot.totemB);
        telemetry.addData("Use Rake", robot.rakeB);
        telemetry.addData("Use Vuforia", robot.vuforiaB);


        //Set up our IMU telemetry dashboard
        if(robot.BNO) {
            composeIMUTelemetry();
        }

        //Update the telemetry
        telemetry.update();

        //Wait for the Pressing of the Start Button
        waitForStart();

        //Reset the game clock to zero in Start()
        runtime.reset();

        //Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Activate the usage of the drive train
            if(robot.drive) {
                //Set the calculated motor speed to a usable variable
                returnedDriveSpeed[0] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[0];
                returnedDriveSpeed[1] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[1];
                returnedDriveSpeed[2] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[2];
                returnedDriveSpeed[3] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)[3];

                //Set each motor's power to the value calculated
                robot.drive1.setPower(returnedDriveSpeed[0]);
                robot.drive2.setPower(returnedDriveSpeed[1]);
                robot.drive3.setPower(returnedDriveSpeed[2]);
                robot.drive4.setPower(returnedDriveSpeed[3]);

                //Drive Speed Telemetry
                if (driveTelemetry) {
                    telemetry.addData("Drive 1 Speed", returnedDriveSpeed[0]);
                    telemetry.addData("Drive 2 Speed", returnedDriveSpeed[1]);
                    telemetry.addData("Drive 3 Speed", returnedDriveSpeed[2]);
                    telemetry.addData("Drive 4 Speed", returnedDriveSpeed[3]);
                    telemetry.addData("Joystick Angle", joystickAngle);
                    telemetry.addData("Joystick X", gamepad1.left_stick_x);
                    telemetry.addData("Joystick Y", gamepad1.left_stick_y);

                }
            }

            if(robot.liftB) {
                robot.lift.setPower(gamepad2.left_stick_y);

                telemetry.addData("Gamepad 2 Right Stick", gamepad2.right_stick_y);
                telemetry.addData("Lift Power", robot.lift.getPower());
                telemetry.addData("Lift Pos", robot.lift.getCurrentPosition());
            }

            if(robot.totemB) {
                if (gamepad2.right_stick_y > .2) {
                    totemPos = totemPos - 0.02;
                }else if (gamepad2.right_stick_y < -.2) {
                    totemPos = totemPos + 0.02;
                };



                robot.totem.setPosition(totemPos);
                telemetry.addData("totem y",gamepad2.left_stick_y);
                telemetry.addData("Totem Pos", totemPos);
            }
            //Update the Telemetry
            telemetry.update();
        }
    }

    //Calculate the speeds of each of the motors based on joystick position
    private double[] driveSpeed(double x1, double y1, double x2){

        //Create a new double which will be the value returned
        double[] output = new double[4];

        //A variable to clip the outputs before sending them to the motors
        double clip = 1.00;

        //Clip the joystick values to prevent error
        /*if(x1<0.05 && x1>-0.05){x1=0.00;}
        if(y1<0.05 && y1>-0.05){y1=0.00;}*/
        if(x2<0.05 && x2>-0.05){x2=0.00;}

        y1=-y1;

        joystickAngle = Math.atan2(y1,x1);

        //Convert left joystick values to motor powers
        if((joystickAngle > Math.PI/4) && (joystickAngle < 3*Math.PI/4)){
            output[0] = -y1;
            output[1] = 0;
            output[2] = y1;
            output[3] = 0;
        }else if((joystickAngle > 3*Math.PI/4) || (joystickAngle < -3*Math.PI/4)){
            output[0] = 0;
            output[1] = -x1;
            output[2] = 0;
            output[3] = x1;
        }else if((joystickAngle > -3*Math.PI/4) && (joystickAngle < -Math.PI/4)){
            output[0] = -y1;
            output[1] = 0;
            output[2] = y1;
            output[3] = 0;
        }else if((joystickAngle > -Math.PI/4) && (joystickAngle < Math.PI/4)){
            output[0] = 0;
            output[1] = -x1;
            output[2] = 0;
            output[3] = x1;
        }else{
            output[0] = 0;
            output[1] = 0;
            output[2] = 0;
            output[3] = 0;
        }

        //When the right joystick is moved side to side, spin the robot in that direction
        if(x2 != 0.00){
            output[0] = output[0] + x2/2;
            output[1] = output[1] + x2/2;
            output[2] = output[2] + x2/2;
            output[3] = output[3] + x2/2;
        }

        //Set the highest motor power as the clipping value to divide all the power by...
        if (output[0]>clip || output[0]<-clip){clip = Math.abs(output[0]);}
        if (output[1]>clip || output[1]<-clip){clip = Math.abs(output[1]);}
        if (output[2]>clip || output[2]<-clip){clip = Math.abs(output[2]);}
        if (output[3]>clip || output[3]<-clip){clip = Math.abs(output[3]);}

        //...to send in doubles less than one.
        output[0] = output[0]/clip;
        output[1] = output[1]/clip;
        output[2] = output[2]/clip;
        output[3] = output[3]/clip;

        //Return the manipulated variable
        return output;
    }

    //Create the IMU telemetry
    private void composeIMUTelemetry() {

        //Create neccessary values once to save the hardware
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = robot.imu.getGravity();
        }
        });

        //Show the status of the IMU itself
        telemetry.addLine()
                .addData("status", new Func<String>(){@Override public String value(){return robot.imu.getSystemStatus().toShortString();}})
                .addData("calib", new Func<String>(){@Override public String value(){return robot.imu.getCalibrationStatus().toString();}});

        //Show the useful angles found by the IMU
        telemetry.addLine()
                .addData("heading", new Func<String>(){@Override public String value(){return formatAngle(angles.angleUnit, angles.firstAngle);}})
                .addData("roll", new Func<String>(){@Override public String value(){return formatAngle(angles.angleUnit, angles.secondAngle);}})
                .addData("pitch", new Func<String>(){@Override public String value(){return formatAngle(angles.angleUnit, angles.thirdAngle);}});

        //Show acceleration data
        telemetry.addLine().addData("mag", new Func<String>() {
                    @Override public String value(){return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel + gravity.yAccel*gravity.yAccel + gravity.zAccel*gravity.zAccel));}});
    }

    //Format the angle given by the IMU into a useful number
    private String formatAngle(AngleUnit angleUnit, double angle){return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));}

    //Format the angle given by the IMU into degrees
    private String formatDegrees(double degrees){return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));}

}
