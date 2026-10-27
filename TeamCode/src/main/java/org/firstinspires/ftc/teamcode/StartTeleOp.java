package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "StartTeleOp2Test")
public class StartTeleOp extends LinearOpMode {

    private static final double deadband = 0.08;
    private static final double maxPower = 0.85;

    @Override
    public void runOpMode() {

        // defines deadband as 8% of stick travel

        // giving all motors references
        DcMotor BLMotor = hardwareMap.get(DcMotor.class, "MotorBL");
        DcMotor BRMotor = hardwareMap.get(DcMotor.class, "MotorBR");
        DcMotor FRMotor = hardwareMap.get(DcMotor.class, "MotorFR");
        DcMotor FLMotor = hardwareMap.get(DcMotor.class, "MotorFL");

        IMU imu;

        boolean lastAState = false; // Checks if a button is pressed already

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

            // left stick controls and powers left motors and vice versa for right motors (without deadband)

            double drivePowerLeft = gamepad1.left_stick_y;
            double drivePowerRight = gamepad1.right_stick_y;

            // modified deadband???
            double moddedPowerLeft = scaleValue(drivePowerLeft, Math.signum(drivePowerLeft));
            double moddedPowerRight = scaleValue(drivePowerRight, Math.signum(drivePowerRight));

            // Uses deadband in order to make movement more smooth ( IF USING THIS DO NOT USE CODE ABOVE)
            /*

            double sigLeft = Math.signum(drivePowerLeft);
            double sigRight = Math.signum(drivePowerRight);

            double scaledRight = scaleValue(drivePowerRight, sigRight);
            double scaledLeft = scaleValue(drivePowerLeft, sigLeft);

             */


            for (DcMotor m : leftMotors)
            {
                m.setPower(moddedPowerLeft);
            }

            for (DcMotor m : rightMotors)
            {
                m.setPower(-moddedPowerRight);
            }

            // add statistics and data to debug log on driver hub
            telemetry.addData("Motor Power Left Side (w/ deadband)", "%.2f", drivePowerLeft);
            telemetry.addData("Motor Power Right Side (w/ deadband)", "%.2f", drivePowerRight);
            telemetry.addData("Left Joystick Input", "%.2f", gamepad1.left_stick_y);
            telemetry.addData("Right Joystick Input", "%.2f", gamepad1.right_stick_y);
            telemetry.addData("modified power left", "%.2f", moddedPowerLeft);
            telemetry.addData("modified power right", "%.2f", moddedPowerRight);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }

    public static double scaleValue(double value, double sign)
    {
        double limitToMaxVal = value * maxPower;
        if (Math.abs(value) < deadband)
        {
            return 0.0;
        }
        else {
            double newVal = (Math.abs(limitToMaxVal) - deadband) / (1.0 - deadband);
            return newVal * sign;
        }
    }

}