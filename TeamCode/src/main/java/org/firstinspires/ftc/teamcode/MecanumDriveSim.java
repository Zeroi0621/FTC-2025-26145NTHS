package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name = "MecanumDriveSim", group="Linear OpMode")
public class MecanumDriveSim extends LinearOpMode {

    // the cool motors
    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor back_left_motor;
    DcMotor back_right_motor;
    private CRServo leftIntake;
    private CRServo rightIntake;
    private DcMotor arm_motor;
    private DcMotor arm_long;
    // Speed multiplier
    double speedMultiplier = 0.0;
    double intakePower = 0.0;

    // Variables to track previous button states
    boolean previousLB = false;
    boolean previousA = false;
    boolean previousB = false;
    boolean previousY = false;
    boolean previousX = false;

    @Override
    public void runOpMode() {
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_long = hardwareMap.get(DcMotor.class, "arm_long");
        // Initialize the hardware map
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Set the motors to run without encoders for free movement (edit for fun or if needed)
        front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Invert the right-side motors (if needed)
        front_right_motor.setDirection(DcMotor.Direction.REVERSE);
        back_right_motor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for game to start
        waitForStart();
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);

        // Main loops
        while (opModeIsActive()) {
            if (gamepad2.dpad_down) {
                arm_motor.setPower(-1);
            }
            if (gamepad2.dpad_up) {
                arm_motor.setPower(1);
            }
            if (arm_motor.getCurrentPosition() > 3200) {
                arm_motor.setPower(-0.2);
            } else if (arm_motor.getCurrentPosition() < 2000) {
                arm_motor.setPower(0.2);
            }
            if (gamepad1.left_bumper == false){
                if (gamepad2.x) {
                    intakePower = -0.3;
                }
                if (gamepad2.b) {
                    intakePower = 1;
                }
                if (gamepad2.y) {
                    intakePower = 0;
                }
            }

            // Update arm_long controls to use bumpers
            if (gamepad2.left_bumper) {
                arm_long.setPower(1); // Extend the arm
            } else if (gamepad2.right_bumper) {
                arm_long.setPower(-1); // lower the  arm
            } else {
                arm_long.setPower(0.05); // holding power when not moving
            }

            rightIntake.setPower(intakePower);
            leftIntake.setPower(-intakePower);
            arm_motor.setPower(0);

            // Check if LB is pressed, and if it is track the other button press
            if (gamepad1.left_bumper) {
                // Check if LB + A is pressed (and was not pressed last iteration)
                if (gamepad1.a) {
                    speedMultiplier = 0.2; // Set speed to 0.2
                }
                // Check if LB + B is pressed (and was not pressed last iteration)
                else if (gamepad1.b) {
                    speedMultiplier = 0.4; // Set speed to 0.4
                }
                // Check if LB + Y is pressed (and was not pressed last iteration)
                else if (gamepad1.y) {
                    speedMultiplier = 0.6; // Set speed to 0.6
                }
                // Check if LB + X is pressed (and was not pressed last iteration)
                else if (gamepad1.x) {
                    speedMultiplier = 0.8; // Set speed to 0.8
                }
                else if (gamepad1.right_bumper){
                    speedMultiplier = 1.0;
                }
            }

            // the inputs for the gamepad
            double drive = -gamepad1.left_stick_y * speedMultiplier;  // Forward/Backward (left stick Y-axis)
            double strafe = gamepad1.left_stick_x * speedMultiplier;  // Strafing (left stick X-axis)
            double turn = gamepad1.right_stick_x * speedMultiplier;   // Turning (right stick X-axis)

            // Calculate the power for each wheel
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // Normalize the wheel speeds so they dont go 100%
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

            if (maxPower > 1.0) {
                frontLeftPower = maxPower;
                frontRightPower = maxPower;
                backLeftPower = maxPower;
                backRightPower = maxPower;
            }

            // Set the motor powers
            front_left_motor.setPower(-frontLeftPower);
            front_right_motor.setPower(-frontRightPower);
            back_left_motor.setPower(-backLeftPower);
            back_right_motor.setPower(-backRightPower);


            // Print values for debugging
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.addData("arm", arm_motor.getCurrentPosition());
            telemetry.addData("vipor pos", arm_long.getCurrentPosition());
            telemetry.addData("Speed Multiplier", speedMultiplier);
            telemetry.update();

            // Store the current state of the buttons for iteration
            previousA = gamepad1.a;
            previousB = gamepad1.b;
            previousY = gamepad1.y;
            previousX = gamepad1.x;

            if (gamepad2.right_bumper == true) {
                if (gamepad2.y == true) {
                    while (true) {
                        arm_motor.setPower(-1);
                    }
                }
            }
        }
    }
}
