package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "StartTeleOp2Test")
public class StartTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        // giving all motors references
        DcMotor BLMotor = hardwareMap.get(DcMotor.class, "MotorBL");
        DcMotor BRMotor = hardwareMap.get(DcMotor.class, "MotorBR");
        DcMotor FRMotor = hardwareMap.get(DcMotor.class, "MotorFR");
        DcMotor FLMotor = hardwareMap.get(DcMotor.class, "MotorFL");

        // separate right and left motors for tank drive
        // allMotors is to configure all at the same time
        DcMotor[] rightMotors = {FRMotor, BRMotor};
        DcMotor[] leftMotors = {FLMotor, BLMotor};
        DcMotor[] allMotors = {BLMotor, BRMotor, FRMotor, FLMotor};

        // make sure all motors are "braked" when no joystick movement on their respective sides.
        for (DcMotor m : allMotors)
        {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // update statistics on driver hub.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // left stick controls and powers left motors and vice versa for right motors
            double drivePowerLeft = gamepad1.left_stick_y;
            double drivePowerRight = gamepad1.right_stick_y;

            for (DcMotor m : leftMotors)
            {
                m.setPower(-drivePowerLeft);
            }

            for (DcMotor m : rightMotors)
            {
                m.setPower(drivePowerRight);
            }

            // add statistics and data to debug log on driver hub
            telemetry.addData("Motor Power Left Side", "%.2f", drivePowerLeft);
            telemetry.addData("Motor Power Right Side", "%.2f", drivePowerRight);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}