package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Channels;

public class ClickerMotorSubsystem extends SubsystemBase {
    private final WPI_TalonSRX limitSwitchMotor;
    private final DigitalInput leftLimit;
    private final DigitalInput rightLimit;

    private float lsmSpeed;

    public ClickerMotorSubsystem() {
        limitSwitchMotor = new WPI_TalonSRX(Channels.LIMIT_SWITCH_LEFT);
        rightLimit = new DigitalInput(Channels.LIMIT_SWITCH_RIGHT);
        leftLimit = new DigitalInput(Channels.LIMIT_SWITCH_LEFT);
        lsmSpeed = (float) 0.15;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("L.S.M. Speed", lsmSpeed);
        SmartDashboard.putBoolean("Left Limit Switch status", leftLimit.get());
        SmartDashboard.putBoolean("Right Limit Switch status", rightLimit.get());
    }

    public Command runClickerMotor() {
        return this.run(() -> {
            final boolean llOnValue = leftLimit.get();
            final boolean rlOnValue = rightLimit.get();
            limitSwitchMotor
                    .set(ControlMode.PercentOutput, getClickerMotorSpeed(llOnValue, rlOnValue));
        });
    }

    // ---------------------------------------------------------------------------------------------
    private float getClickerMotorSpeed(final boolean llOnValue, final boolean rlOnValue) {
        final boolean isClockWise = lsmSpeed > 0;

        // Special case to get the motor in the correct section of rotation by hand.
        if (rlOnValue && llOnValue) {
            lsmSpeed = Math.abs(lsmSpeed);
            return lsmSpeed;
        }

        if ((isClockWise && rlOnValue) || (!isClockWise && llOnValue)) {
            lsmSpeed = -lsmSpeed;
        }

        return lsmSpeed;
    }
}
