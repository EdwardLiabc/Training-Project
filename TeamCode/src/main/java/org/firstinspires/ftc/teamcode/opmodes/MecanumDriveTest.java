package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@TeleOp
public class MecanumDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        Lift lift = new Lift(robot);
        robot.registerSubsystem(mecanumDrive);
        robot.registerSubsystem(lift);
        TelemetryPacket packet = robot.getPacket();

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
//            packet.put("Gamepad Left Stick Y", -gamepad1.left_stick_y);
//            packet.put("Gamepad Left Stick X", -gamepad1.left_stick_x);
//            packet.put("Gamepad Right Stick Y", -gamepad1.right_stick_x);

            // Bumper: boolean, either it is pressed or it isn't
            if (gamepad1.left_bumper) {
                lift.setLiftPower(1);
            }
            // Trigger: float, ranges from 0.0 (not pressed) to 1.0 (fully pressed)
            else {
                lift.setLiftPower(-gamepad1.left_trigger);
            }
        }
    }
}
