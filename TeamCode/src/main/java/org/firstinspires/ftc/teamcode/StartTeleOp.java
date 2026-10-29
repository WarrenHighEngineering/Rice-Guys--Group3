package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "StartTeleOp2Test")
public class StartTeleOp extends LinearOpMode {

    // defining max deadband and power
    private static final double deadband = 0.08;
    private static final double maxPower = 0.85;
    private static double kP;
    private static double kI;
    private static double kD;

    boolean lastAState = false; // Checks if a button is pressed already

    @Override
    public void runOpMode() {

        // Initializing motors (configs in driver hub)
        DcMotor BLMotor = hardwareMap.get(DcMotor.class, "MotorBL");
        DcMotor BRMotor = hardwareMap.get(DcMotor.class, "MotorBR");
        DcMotor FRMotor = hardwareMap.get(DcMotor.class, "MotorFR");
        DcMotor FLMotor = hardwareMap.get(DcMotor.class, "MotorFL");

        // Initializes IMU and its orientation variables
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Starting direction: Is logo facing straight up to the sky? Which way are usb ports facing?
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(params);

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

            if (gamepad1.square && !lastAState)
            {
                turnDegrees(90, imu, leftMotors, rightMotors);
            }

            if (gamepad1.circle && !lastAState)
            {
                turnDegrees(-90, imu, leftMotors, rightMotors);
            }

            /* // left stick controls and powers left motors and vice versa for right motors (without deadband)
            double drivePowerLeft = gamepad1.left_stick_y;
            double drivePowerRight = gamepad1.right_stick_y;
            */

            double drivePowerLeft;
            double drivePowerRight;

            if (gamepad1.left_bumper)
            {
                drivePowerLeft = 1.00;
            }
            else if (gamepad1.left_trigger_pressed)
            {
                drivePowerLeft = -1.00;
            }
            else
            {
                drivePowerLeft = 0;
            }

            if (gamepad1.right_bumper)
            {
                drivePowerRight = 1.00;
            }
            else if (gamepad1.right_trigger_pressed)
            {
                drivePowerRight = -1.00;
            }
            else
            {
                drivePowerRight = 0;
            }


            // modified deadband???
            double moddedPowerLeft = scaleValue(drivePowerLeft, Math.signum(drivePowerLeft));
            double moddedPowerRight = scaleValue(drivePowerRight, Math.signum(drivePowerRight));

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

    private void turnDegrees(double targetAngle, IMU imu, DcMotor[] leftMotors, DcMotor[] rightMotors) {
        // Reset IMU yaw orientation to 0
        imu.resetYaw();

        double turnPower = 0.5; // Turn speed (0.0 to 1.0)
        double currentHeading = getHeading(imu);
        double targetHeading = currentHeading + targetAngle;

        // Determine spin direction: positive angle = left turn, negative = right turn
        // ? means if/else so  - variable = (condition) ? true (do this) : false (do this)
        double leftPower = (targetAngle > 0) ? turnPower : -turnPower; // if target angle is greater than 0, turn left wheels back (go left). else go right.
        double rightPower = (targetAngle > 0) ? turnPower : -turnPower; // if target angle is greater than 0, turn right wheels back (go left). else go right.

        // Drive motors in opposite directions until heading reaches target angle
        // the > 2.0 is just for better control with the rotation
        while (opModeIsActive() && Math.abs(getHeading(imu) - targetHeading) > 2.0) {
            for (DcMotor m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotor m : rightMotors) {
                m.setPower(rightPower); // Matching inverted polarity from main loop
            }

            telemetry.addData("Turning to", targetHeading);
            telemetry.addData("Current Heading", getHeading(imu));
            telemetry.update();
        }

        // Stop all motors after turn finishes
        for (DcMotor m : leftMotors) m.setPower(0);
        for (DcMotor m : rightMotors) m.setPower(0);
    }

    // Finds what direction we are rotated relative to the starting direction.
    private double getHeading(IMU imu)
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}