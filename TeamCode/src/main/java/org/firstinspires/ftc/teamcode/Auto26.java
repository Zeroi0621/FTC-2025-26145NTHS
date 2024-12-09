package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "Auto26", group = "Autonomous")
public class Auto26 extends LinearOpMode {
    public class arm {
        private DcMotor arm_motor;

        public arm(HardwareMap hardwareMap) {
            arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
            arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class armUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    arm_motor.setPower(0.8);
                    initialized = true;
                }

                // checks arm's current position
                double pos = arm_motor.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos < 3000.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    arm_motor.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action armUp() {
            return new armUp();
        }
        // within the Lift class
        public class armDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm_motor.setPower(-0.8);
                    initialized = true;
                }

                double pos = arm_motor.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    arm_motor.setPower(0);
                    return false;
                }
            }
        }

        public Action armDown() {
            return new armDown();
        }
    }

    // claw class
    public class intake {
        private CRServo leftIntake;
        private CRServo rightIntake;

        public intake(HardwareMap hardwareMap) {
            leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
            rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        }

        // within the Claw class
        public class In implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightIntake.setPower(-1.0);
                rightIntake.setPower(1.0);
                return false;
            }
        }

        public Action In() {
            return new In();
        }

        public class Out implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightIntake.setPower(1.0);
                rightIntake.setPower(-1.0);
                return false;
            }
        }

        public Action Out() {
            return new Out();
        }
    }
    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        intake rightIntake = new intake(hardwareMap);
        // make a Lift instance
        arm arm_motor = new arm(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;
        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);

        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }
        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        arm_motor.armUp(),
                        arm_motor.armDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}