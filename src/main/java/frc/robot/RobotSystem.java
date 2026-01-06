package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.BlackMotorSubsystem;
import frc.robot.subsystems.ClickerMotorSubsystem;
import frc.robot.subsystems.PneumaticHubSubsystem;
import frc.robot.subsystems.RedServoSubsystem;
import frc.robot.subsystems.WhiteServoSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotSystem {
        private final SwerveSubsystem drivebase;
        private final BlackMotorSubsystem blackMotorSubsystem;
        @SuppressWarnings("unused") // - directly
        private final WhiteServoSubsystem whiteServoSubsystem;
        @SuppressWarnings("unused") // - directly
        private final RedServoSubsystem redServoSubsystem;
        @SuppressWarnings("unused") // - directly
        private final PneumaticHubSubsystem pneumaticHubSubsystem;
        @SuppressWarnings("unused") // - directly
        private final ClickerMotorSubsystem clickerMotorSubsystem;
        @SuppressWarnings("unused") // - directly
        private final OperatorInput operatorInput;

        public RobotSystem() {
                drivebase = new SwerveSubsystem(Filesystem.getDeployDirectory());
                blackMotorSubsystem = new BlackMotorSubsystem();
                whiteServoSubsystem = new WhiteServoSubsystem();
                redServoSubsystem = new RedServoSubsystem();
                pneumaticHubSubsystem = new PneumaticHubSubsystem();
                clickerMotorSubsystem = new ClickerMotorSubsystem();
                operatorInput = new OperatorInput(drivebase, blackMotorSubsystem);

                DriverStation.silenceJoystickConnectionWarning(true);
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}