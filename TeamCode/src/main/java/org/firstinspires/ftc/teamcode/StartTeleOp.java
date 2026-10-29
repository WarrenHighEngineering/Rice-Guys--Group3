package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "StartTeleOp2Test")
public class StartTeleOp extends LinearOpMode {

    // defining max deadband and power
    private static final double deadband = 0.08;
    private static final double maxPower = 0.9;
    private static final double maxTurnPower = 0.50;
    private static final double kP = 0.015;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double turnTolerance = 1.0; // degrees of tolerance
    private static final double integralZone = 10.0; // degree range to use integral
    private static final double turnTimeout = 1.5; // seconds until turn is timed out
    private static final int settleLoops = 5; // loops in tolerance
    private static final int turnDirection = 1; // change to -1 if robot spins wrong way

    boolean lastCircleState = false; // Checks if a button is pressed already
    boolean lastSquareState = false;

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

            if (gamepad1.square && !lastSquareState)
            {
                turnDegrees(90, imu, leftMotors, rightMotors);
            }

            if (gamepad1.circle && !lastCircleState)
            {
                turnDegrees(-90, imu, leftMotors, rightMotors);
            }
            lastSquareState = gamepad1.square;
            lastCircleState = gamepad1.circle;

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

    // an autonomous function that uses commands to turn some degrees with PID
    private void turnDegrees(double targetAngle, IMU imu, DcMotor[] leftMotors, DcMotor[] rightMotors) {
        // Reset IMU yaw orientation to 0
        imu.resetYaw();

        double integral = 0;
        double lastError = targetAngle;
        int settledCount = 0;

        ElapsedTime timeoutTimer = new ElapsedTime(); // has a timer to see if action is taking longer than the timeout time (3 s)
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive() && timeoutTimer.seconds() < turnTimeout) {
            double heading = getHeading(imu); // gets the current rotation of the robot
            double error = angleWrap(targetAngle - heading); // The difference between target point and current heading

            /*
            * dt is kinda like a time tracker. We use it to track integral and derivative values over time
            * execution loops by FTC fluctuate so its good to keep a constant time elapse
            * the 0.001 is to prevent divide by 0 errors (Math.max takes the highest num from two vals)
            */

            double dt = Math.max(loopTimer.seconds(), 0.001);
            loopTimer.reset();

            // power based on distance to target
            double p = kP * error;

            if (Math.abs(error) < integralZone) {
                integral += error * dt; // dt
                // while in the integral zone, integral builds up.
            } else {
                integral = 0;
            }

            double i = kI * integral;

            double d = kD * (error - lastError) / dt;
            lastError = error;

            // restricts power to fluctuating within a certain range.
            double power = Range.clip(p + i + d, -maxTurnPower, maxTurnPower);

            // settled count is used to limit oscillations with the robot when it tries to reach the desired angle
            // letting it oscillate for an amount of times (settleLoops) really makes sure it reaches and stablizes at the angle
            if (Math.abs(error) < turnTolerance) {
                settledCount++;

                if (settledCount > settleLoops) break;
            } else {
                settledCount = 0;
            }

            // Stop all motors after turn finishes
            for (DcMotor m : leftMotors) m.setPower(power * turnDirection);
            for (DcMotor m : rightMotors) m.setPower(power * turnDirection);

            telemetry.addData("Target", "%.2f", targetAngle);
            telemetry.addData("Current Heading", "%.2f", heading);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("P / I / D", "%.3f / %.3f / %.3f" , p, i, d);
            telemetry.addData("Power", "%.3f", power);
            telemetry.update();
        }
    }

    private void moveFwdAmt(double distance, IMU imu, DcMotor[] leftMotors, DcMotor[] rightMotors)
    {

    }

    // Finds what direction we are rotated relative to the starting direction.
    private double getHeading(IMU imu)
    {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // keeps angles between -180 and 180
    private double angleWrap(double degrees)
    {
        while (degrees > 180) degrees -= 360; // if angle is greater than 180, subtract 360
        while (degrees < -180) degrees += 360; // if angle is less than -180, add 360

        return degrees;
    }

}