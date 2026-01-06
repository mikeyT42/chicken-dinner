package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Channels;

public class WhiteServoSubsystem extends SubsystemBase {
    private final Servo whiteServo;
    private boolean increasingWhiteServo;

    public WhiteServoSubsystem() {
        whiteServo = new Servo(Channels.WHITE_SERVO_CHANNEL);
        whiteServo.set(0);
        increasingWhiteServo = true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("White Motor Angle", whiteServo.getAngle());
    }

    // ---------------------------------------------------------------------------------------------
    public Command runWhiteServo() {
        return this.run(() -> whiteServo.setAngle(getAngle()));
    }

    // ---------------------------------------------------------------------------------------------
    private double getAngle() {
        final float maxAngle = 80;
        float currentAngle = (float) whiteServo.getAngle();

        if (currentAngle >= maxAngle) {
            increasingWhiteServo = false;
        }
        if (currentAngle <= 0.0) {
            increasingWhiteServo = true;
        }

        if (increasingWhiteServo) {
            currentAngle += 5.0;
        } else {
            currentAngle -= 5.0;
        }

        return currentAngle;
    }
}
