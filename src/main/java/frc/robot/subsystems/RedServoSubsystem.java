package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Channels;

public class RedServoSubsystem extends SubsystemBase {
    private final Servo redServo;
    private boolean increasingRedServo;

    public RedServoSubsystem() {
        redServo = new Servo(Channels.RED_SERVO_CHANNEL);
        redServo.set(180.0);
        increasingRedServo = true;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Red Motor Angle", redServo.getAngle());
    }

    // -----------------------------------------------------------------------------------------------------------------
    public Command runWhiteServo() {
        return this.run(() -> redServo.setAngle(getAngle()));
    }

    // -----------------------------------------------------------------------------------------------------------------
    private double getAngle() {
        final float maxAngle = 180;
        float currentAngle = (float) redServo.getAngle();

        if (currentAngle >= maxAngle) {
            increasingRedServo = false;
        }
        if (currentAngle <= 0.0) {
            increasingRedServo = true;
        }

        if (increasingRedServo) {
            currentAngle += 5.0;
        } else {
            currentAngle -= 5.0;
        }

        return currentAngle;
    }
}
