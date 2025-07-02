package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoMovementGeneralization extends LinearOpMode {
    Servo servo;

    @Override
    public void runOpMode() {
        double degrees_of_movement = 90; // example degrees to move

        servo = hardwareMap.get(Servo.class, "sm5");

        waitForStart();

        while (opModeIsActive()) {
            double intended_movement = degreesToPosition(degrees_of_movement);

            double current_position = servo.getPosition();

            moveToPosition(intended_movement);

            telemetry.addData("Servo position is:", current_position);
            telemetry.update();

            sleep(100); 
        }
    }

    public void moveToPosition(double position) {
        servo.setPosition(position);
    }

    // Converts degrees (0 to 270) to servo position (0.0 to 1.0)
    public double degreesToPosition(double degrees) {
        return degrees / 270.0;
    }
}       
