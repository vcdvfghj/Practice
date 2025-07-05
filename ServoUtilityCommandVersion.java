package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoUtilityCommandVersion extends SubsystemBase {
    private Servo cs;
    public double targetServoPos;

    public ServoUtilityCommandVersion(Servo cs) {
        this.cs = cs;
        if (targetServoPos > 1) {
            throw new IllegalArgumentException("Your selected position is out of the range of a servo");
        }
    }

    public void setPosition(double position) {
        cs.setPosition(position);
        }

    public double setRange(double minAngle, double maxAngle){
        double range = maxAngle - minAngle;
        return range;
    }

//    public double setRange(double range){
//        return range;
//    }

    public double setZeroPosition(double angleOfCalibration, double servoPosAtCalibration, double range) {
        double zeroPos = (servoPosAtCalibration - convertDegreestoPos(angleOfCalibration, range));
        return zeroPos;
    }

    public double convertDegreestoPos(double degrees, double input_range){
        double posInDegrees = degrees/ input_range;
        return posInDegrees;
    }

    public double convertPostoDegrees(double pos, double input_range){
        double DegreesInPos = pos * input_range;
        return DegreesInPos;
    }


    public double findTargetServoPos(double targetDegrees, double range, double zeroPosition) {
        double targetservoPos = convertDegreestoPos(targetDegrees, range) - zeroPosition;
        return targetservoPos;
    }

    public Command moveServo(double targetServoPos) {
        return new SequentialCommandGroup() {
            {
                addCommands(new InstantCommand(() -> cs.setPosition(targetServoPos)));
            }
        };
    }

    public Command moveServoDeg(double targetDegrees, double range) {
        return new SequentialCommandGroup() {
            {
                addCommands(new InstantCommand(() -> cs.setPosition(convertDegreestoPos(targetDegrees, range))));
            }
        };
    }
}


//moveServo(findTargetServoPos(targetDegrees, setRange(range), setZeroPosition(angleOfCalibration, servoPosAtCalibration, setRange(range)))






