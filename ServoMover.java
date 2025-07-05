package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp
@Config
public class ServoMover extends LinearOpMode {
    public static double targetServoPos;
    public static double targetDegrees = 0;
    public static double minAngle = 0;
    public static double maxAngle = 0;

    public static double angleOfCalibration;
    public static double servoPosAtCalibration;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() {
        CommandScheduler commandScheduler = CommandScheduler.getInstance();

        Servo cs = hardwareMap.get(Servo.class, "cs5");


        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        ServoUtilityCommandVersion utility = new ServoUtilityCommandVersion(cs);
        waitForStart();
        while (opModeIsActive()) {
            targetServoPos = utility.findTargetServoPos(targetDegrees, utility.setRange(minAngle, maxAngle), utility.setZeroPosition(angleOfCalibration, servoPosAtCalibration, utility.setRange(minAngle, maxAngle)));
            targetDegrees = utility. convertPostoDegrees(targetServoPos, utility.setRange(minAngle, maxAngle));
            commandScheduler.schedule(utility.moveServoDeg(targetDegrees, utility.setRange(minAngle, maxAngle)));
        commandScheduler.run();

        telemetry.update();

        }
    }
}
