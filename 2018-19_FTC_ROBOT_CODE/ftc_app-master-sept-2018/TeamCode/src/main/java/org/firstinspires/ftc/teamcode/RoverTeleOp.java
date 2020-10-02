package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class RoverTeleOp extends LinearOpMode {

    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();

    //Set the Hardware Map as the One Specified
    private RoverHardware robot = new RoverHardware();
    private double[] motorSpeed;

    @Override
    public void runOpMode() {

        //Initialize the Hardware Map
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait for the Pressing of the Start Button
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            motorSpeed[0] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad2.left_stick_x)[0];
            motorSpeed[1] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad2.left_stick_x)[1];
            motorSpeed[2] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad2.left_stick_x)[2];
            motorSpeed[3] = driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad2.left_stick_x)[3];

            robot.drive1.setPower(motorSpeed[0]);
            robot.drive1.setPower(motorSpeed[1]);
            robot.drive1.setPower(motorSpeed[2]);
            robot.drive1.setPower(motorSpeed[3]);

            telemetry.addData("Drive 1 Speed", motorSpeed[0]);
            telemetry.addData("Drive 2 Speed", motorSpeed[1]);
            telemetry.addData("Drive 3 Speed", motorSpeed[2]);
            telemetry.addData("Drive 4 Speed", motorSpeed[3]);
            telemetry.update();

        }
    }

    private double[] driveSpeed(double x1, double y1, double x2){

        //Create a new double which will be the value returned
        double[] output = new double[4];

        if(x1==1.00 && y1==0.00){
            output[0]=1.00;
            output[1]=0.00;
            output[2]=-1.00;
            output[3]=0.00;
        }else if(x1==-1.00 && y1==0.00){
            output[0]=-1.00;
            output[1]=0.00;
            output[2]=1.00;
            output[3]=0.00;
        }else if(y1==1.00 && x1==0.00){
            output[0]=0.00;
            output[1]=1.00;
            output[2]=0.00;
            output[3]=-1.00;
        }else if(y1==-1.00 && x1==0.00){
            output[0]=0.00;
            output[1]=-1.00;
            output[2]=0.00;
            output[3]=1.00;
        }else{
            //
            output[0]=0.00;
            output[1]=0.00;
            output[2]=0.00;
            output[3]=0.00;
        }

        return output;

    }
}
