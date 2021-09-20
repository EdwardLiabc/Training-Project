package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

public class Lift implements Subsystem {
    private DcMotorEx liftMotor;
    private double liftPower;

    public Lift (Robot robot) {
        liftMotor = robot.getMotor("liftMotor");
        liftPower = 0.0;
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
    }

    @Override
    public void update(TelemetryPacket packet) {
        liftMotor.setPower(liftPower);

        packet.put("Lift Power", liftPower);
    }
}
