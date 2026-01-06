// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.BlackMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class OperatorInput {
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SwerveSubsystem drivebase;
    private final BlackMotorSubsystem blackMotorSubsystem;

    // Input streams for swerve
    private final SwerveInputStream driveAngularVelocity;
    private final SwerveInputStream driveDirectAngle;
    private final SwerveInputStream driveRobotOriented;
    private final SwerveInputStream driveAngularVelocityKeyboard;
    private final SwerveInputStream driveDirectAngleKeyboard;

    public OperatorInput(SwerveSubsystem drivebase, BlackMotorSubsystem blackMotorSubsystem) {
        this.drivebase = drivebase;
        this.blackMotorSubsystem = blackMotorSubsystem;

        // Initialize input streams
        driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> driverXbox.getLeftY() * -1,
                () -> driverXbox.getLeftX() * -1)
                .withControllerRotationAxis(driverXbox::getRightX)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        driveDirectAngle = driveAngularVelocity.copy()
                .withControllerHeadingAxis(driverXbox::getRightX,
                        driverXbox::getRightY)
                .headingWhile(true);

        driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                .allianceRelativeControl(false);

        driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> -driverXbox.getLeftY(),
                () -> -driverXbox.getLeftX())
                .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                .withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                        () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
                .headingWhile(true)
                .translationHeadingOffset(true)
                .translationHeadingOffset(Rotation2d.fromDegrees(0));

        // Configure all bindings
        configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        driverXbox.start().whileTrue(Commands.none());
        driverXbox.back().whileTrue(Commands.none());
        driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverXbox.rightBumper().onTrue(Commands.none());

        // New bindings for BlackMotorSubsystem using triggers
        new Trigger(() -> driverXbox.getRightTriggerAxis() > OperatorConstants.DEADBAND)
                .whileTrue(blackMotorSubsystem.runMotor(() -> driverXbox.getRightTriggerAxis()));

        new Trigger(() -> driverXbox.getLeftTriggerAxis() > OperatorConstants.DEADBAND)
                .whileTrue(blackMotorSubsystem.runMotor(() -> -driverXbox.getLeftTriggerAxis()));

        // Put trigger values to dashboard (from user's snippet)
        SmartDashboard.putNumber("RT angle", driverXbox.getRightTriggerAxis());
    }

    // Optional: Expose the controller if needed elsewhere
    public CommandXboxController getDriverXbox() {
        return driverXbox;
    }
}