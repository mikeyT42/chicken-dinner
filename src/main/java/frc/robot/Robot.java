package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
	@SuppressWarnings("unused") // - directly
	private RobotSystem robotSystem;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	public Robot() {
		if (super.isAutonomous()) {
			System.out.println("No auto allowed.");
			System.exit(1);
		}

		robotSystem = new RobotSystem();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics that you want ran during disabled, autonomous, teleoperated
	 * and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		if (super.isAutonomous()) {
			System.out.println("No auto allowed.");
			System.exit(1);
		}

		// You run the swerve drive stuff.
		CommandScheduler.getInstance().run();
	}

}
