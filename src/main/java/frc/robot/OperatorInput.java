package frc.robot;

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
        @SuppressWarnings("unused")
        private final SwerveInputStream driveDirectAngle;
        @SuppressWarnings("unused")
        private final SwerveInputStream driveRobotOriented;

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

                // Configure all bindings
                configureBindings();
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                driverXbox.start().whileTrue(Commands.none());
                driverXbox.back().whileTrue(Commands.none());
                driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                driverXbox.rightBumper().onTrue(Commands.none());

                // New bindings for BlackMotorSubsystem using triggers
                new Trigger(() -> driverXbox.getRightTriggerAxis() > 0)
                                .whileTrue(blackMotorSubsystem.runMotor(() -> driverXbox.getRightTriggerAxis()));

                new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0)
                                .whileTrue(blackMotorSubsystem.runMotor(() -> -driverXbox.getLeftTriggerAxis()));

                // Put trigger values to dashboard (from user's snippet)
                SmartDashboard.putNumber("RT angle", driverXbox.getRightTriggerAxis());
        }

        // Optional: Expose the controller if needed elsewhere
        public CommandXboxController getDriverXbox() {
                return driverXbox;
        }
}