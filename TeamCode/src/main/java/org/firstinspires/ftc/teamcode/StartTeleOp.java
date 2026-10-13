package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "StartTeleOp1Test")
public class StartTeleOp extends LinearOpMode {
    // Create variables here


    @Override
    public void runOpMode() {

        DcMotor BLMotor = hardwareMap.get(DcMotor.class, "MotorLeft");
        DcMotor BRMotor = hardwareMap.get(DcMotor.class, "MotorRight");
        DcMotor[] allMotors = {BLMotor, BRMotor};

        for (DcMotor m : allMotors)
        {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData("Motor Power", test_motor.getPower());

            double drivePower = -gamepad1.left_stick_y;

            for (DcMotor m : allMotors)
            {
                m.setPower(drivePower);
            }

            telemetry.addData("Motor Power", "%.2f", drivePower);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}