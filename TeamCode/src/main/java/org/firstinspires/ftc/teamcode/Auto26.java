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
        private DcMotor arm_long;

        public arm(HardwareMap hardwareMap) {
            arm_long = hardwareMap.get(DcMotor.class, "arm_long");
            arm_long.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm_long.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class armUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    arm_long.setPower(1);
                    initialized = true;
                }

                // checks arm's current position
                double pos = arm_long.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos < 2437.5) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    arm_long.setPower(0.05);
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
                telemetry.addData("viper slide", arm_long.getCurrentPosition());
                telemetry.update();
                if (!initialized) {
                    arm_long.setPower(-1);
                    initialized = true;
                }

                double pos = arm_long.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos > 350) {

                    return true;
                } else {
                    arm_long.setPower(0.05);
                    return false;
                }
            }
        }

        public Action armDown() {
            return new armDown();
        }
        public class armDownUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                telemetry.addData("viper slide", arm_long.getCurrentPosition());
                telemetry.update();
                if (!initialized) {
                    arm_long.setPower(1);
                    initialized = true;
                }

                double pos = arm_long.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos < 200) {
                    return true;
                } else {
                    arm_long.setPower(0.05);
                    return false;
                }
            }
        }

        public Action armDownUp() {
            return new armDownUp();
        }
        public class armStop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                telemetry.addData("viper slide", arm_long.getCurrentPosition());
                telemetry.update();
                if (!initialized) {
                    arm_long.setPower(-1);
                    initialized = true;
                }

                double pos = arm_long.getCurrentPosition();
                packet.put("armPos", pos);
                if (pos > 1838) {
                    return true;
                } else {
                    arm_long.setPower(0.05);
                    return false;
                }
            }
        }

        public Action armStop() {
            return new armStop();
        }
    }
    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-20, 60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        arm arm_long = new arm(hardwareMap);
        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0, arm_long.armUp())
                .strafeTo(new Vector2d(-3, 32))
                .stopAndAdd(arm_long.armStop())
                .strafeTo(new Vector2d(-3, 35))
                //clipped 1
                .strafeTo(new Vector2d(-36, 35))
                .afterTime(0, arm_long.armDown())
                .strafeTo(new Vector2d(-36, 10))
                .strafeTo(new Vector2d(-45, 10))
                .strafeTo(new Vector2d(-45, 55))
                .strafeTo(new Vector2d(-45, 10))
                .strafeTo(new Vector2d(-55, 10))
                .strafeTo(new Vector2d(-55, 55))
                .strafeToSplineHeading(new Vector2d(-55, 40), Math.toRadians(0))
                //pushed back 2
                .strafeTo(new Vector2d(-55, 52))
                .strafeTo(new Vector2d(-61, 52))
                .afterTime(0, arm_long.armUp())
                //took clip from wall
                .strafeToSplineHeading(new Vector2d(-3, 40), Math.toRadians(90))
                .strafeTo(new Vector2d(-3, 32))
                .stopAndAdd(arm_long.armStop())
                .strafeTo(new Vector2d(-3, 35))
                //clipped # 2
                .afterTime(0, arm_long.armDown())
                .strafeToSplineHeading(new Vector2d(-61, 52), Math.toRadians(0))
                //took clip
                .afterTime(0, arm_long.armUp())
                .strafeToSplineHeading(new Vector2d(-3, 40), Math.toRadians(90))
                .strafeTo(new Vector2d(-3, 32))
                .stopAndAdd(arm_long.armStop())
                .strafeTo(new Vector2d(-3, 35))
                //clipped # 3
                .strafeTo(new Vector2d(-55, 60));

        Action trajectoryActionCloseOut = tab1.endTrajectory().build();
        waitForStart();

        if (isStopRequested()) return;
        Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
        telemetry.update();
    }
}