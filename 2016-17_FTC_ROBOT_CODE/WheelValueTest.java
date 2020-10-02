package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Wheel Value Test", group="Test")
//@Disabled
public class WheelValueTest extends LinearOpMode {

    // Initialize DC motors
    DcMotor motorRight;
    DcMotor motorLeft;

    // Call motors and autonomous actions
    @Override
    public void runOpMode() throws InterruptedException {

        // Retrieve drive DC motors and set directions
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        for (double power=0.0; power<=100.0; power=power+5){
            motorLeft.setPower(power);
            motorRight.setPower(power);
            sleep(5000);
        }

    }

}
