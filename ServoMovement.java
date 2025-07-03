package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Line;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SimpleServo;

@Config
@TeleOp
public class ServoMovement extends LinearOpMode {

    SimpleServo servo;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;





    public static double  target_degrees = 90;
    public static double  range_of_angles = 270;

    public static double angle_of_calibration = 0;
    public static double servoPos_at_calibration = 0;
    public static double zero_position = (servoPos_at_calibration - angle_of_calibration/range_of_angles) / range_of_angles;



    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();

        dashboardTelemetry = dashboard.getTelemetry();

        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);



        servo = new SimpleServo(hardwareMap, "sm5", 0, 270);
        waitForStart();
        while (opModeIsActive()) {
            double target_servoPos = target_degrees/range_of_angles;
            double currentposition = servo.getPosition();
            servo.setPosition(target_servoPos-zero_position);
            if (target_servoPos > 1) {
                throw new IllegalArgumentException("The angle you selected(" + target_degrees + ") is greater than the range that a servo can move");

            }

        }
    }
}
