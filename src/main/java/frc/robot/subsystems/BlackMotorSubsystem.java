// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Channels;

import java.util.function.DoubleSupplier;

public class BlackMotorSubsystem extends SubsystemBase {
    private final SparkMax neoMotor;
    private final RelativeEncoder encoder;

    public BlackMotorSubsystem() {
        neoMotor = new SparkMax(Channels.NEO_CHANNEL, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(25);
        neoMotor.configure(config, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        encoder = neoMotor.getEncoder();
    }

    @Override
    public void periodic() {
        // Telemetry from the user's snippet
        double speed = neoMotor.get();
        double velocity = encoder.getVelocity();
        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Velocity", velocity);
    }

    // High-level method for commands to use
    public void setMotorSpeed(double speed) {
        neoMotor.set(speed);
    }

    // Example command factory: Run motor based on a supplier (e.g., trigger axis)
    public Command runMotor(DoubleSupplier speedSupplier) {
        return this.run(() -> setMotorSpeed(speedSupplier.getAsDouble()));
    }
}