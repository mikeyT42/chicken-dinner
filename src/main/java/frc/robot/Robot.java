// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final short NUM_CHANNELS = 16;

  private double angle;
  private boolean increasing;
  private short currentChannel;

  private final Servo whiteMotor;
  private final Servo redMotor;
  private final SparkMax blackMotor;
  private final Solenoid[] solenoids;
  private final PowerDistribution powerDistribution;
  private final Timer timer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public Robot() {
    redMotor = new Servo(0);
    whiteMotor = new Servo(1);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(30);
    blackMotor = new SparkMax(19, MotorType.kBrushless);
    blackMotor.configure(config, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    powerDistribution = new PowerDistribution(2, ModuleType.kRev);
    timer = new Timer();
    angle = 0.0;
    increasing = true;
    currentChannel = 0;

    final PneumaticHub pneumaticHub = new PneumaticHub(1);
    solenoids = new Solenoid[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
      solenoids[i] = pneumaticHub.makeSolenoid(i);
    }
    pneumaticHub.close();

    timer.start();
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
    SmartDashboard.putNumber("Debug White Motor Angle",
        whiteMotor.getAngle());
    SmartDashboard.putNumber("Debug Red Motor Angle",
        redMotor.getAngle());
    SmartDashboard.putNumber("Debug Current Channel", currentChannel);
    SmartDashboard.putNumber("Total Power via Hub", powerDistribution.getTotalPower());
    SmartDashboard.putNumber("Total Current via Hub", powerDistribution.getTotalCurrent());
    SmartDashboard.putNumber("Voltage via Hub", powerDistribution.getVoltage());

    runBlackMotor();
    runRedWhiteMotor();
    runLightBlink();
  }

  // -----------------------------------------------------------------------------------------------
  private void runBlackMotor() {
    final double speed = blackMotor.get();
    final double output = blackMotor.getAppliedOutput();
    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Output", output);

    final double increaseFactor = 0.003;
    if (output == 0.0 && speed > 0.0)
      blackMotor.set(0);
    if (speed < 0.20)
      blackMotor.set(output + increaseFactor);
  }

  // -----------------------------------------------------------------------------------------------
  private void runRedWhiteMotor() {
    if (increasing) {
      angle += 5.0;
      if (angle >= 180.0) {
        increasing = false;
      }
    } else {
      angle -= 5.0;
      if (angle <= 0.0) {
        increasing = true;
      }
    }

    whiteMotor.setAngle(angle);
    redMotor.setAngle(angle);
  }

  // -----------------------------------------------------------------------------------------------
  void runLightBlink() {
    if (timer.hasElapsed(1.5)) {
      if (currentChannel == NUM_CHANNELS) {
        for (Solenoid solenoid : solenoids) {
          solenoid.set(false);
        }
        currentChannel = 0;
        timer.reset();
        return; // Skip the rest so that we can have blank time.
      }
      solenoids[currentChannel].set(true);
      currentChannel = (short) (currentChannel + 1);
      timer.reset();
    }
  }

  // -----------------------------------------------------------------------------------------------
  private void runScan() {
    System.out.println("Starting SPARK MAX CAN scan (0..63). Keep robot disabled and motors " +
        "disconnected if possible.");
    SmartDashboard.putString("SparkScan", "Starting");

    for (int id = 1; id < 63; id++) {
      SparkMax candidate = new SparkMax(id, MotorType.kBrushless);
      try {
        Timer.delay(0.05);

        double busV = candidate.getBusVoltage();
        if (!Double.isNaN(busV) && busV > 0.0) {
          String msg = String.format("Found SPARK MAX at CAN ID %d — BusVoltage=%.2fV",
              id, busV);
          SmartDashboard.putString("SparkScanFound_" + id, msg);
        }
      } catch (Exception e) {
        SmartDashboard.putString("SparkScan", "Error");
      } finally {
        candidate.close();
      }
    }

    SmartDashboard.putString("SparkScan", "Complete");
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
   * Dashboard, remove all of the chooser code and uncomment the getString line
   * to get the auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structurem below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
