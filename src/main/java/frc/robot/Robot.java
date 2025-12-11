// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DigitalInput;
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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in the TimedRobot documentation. If you change the name of
 * this class or the package after creating this project, you must also update
 * the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // ===============================================================================================
  // PWM
  // ===============================================================================================
  private static final short RED_SERVO_CHANNEL = 0;
  private static final short WHITE_SERVO_CHANNEL = 1;

  // ===============================================================================================
  // CAN
  // ===============================================================================================
  private static final short NEO_CHANNEL = 2;
  private static final short LS_MOTOR_CHANNEL = 4;
  private static final short POWER_DISTRIBUTION_CHANNEL = 3;
  private static final short PNEUMATIC_HUB_CHANNEL = 1;

  // For the pneumatic hub initialization and light blinking.
  private static final short NUM_CHANNELS = 16;

  // ===============================================================================================
  // DIO
  // ===============================================================================================
  private static final short LIMIT_SWITCH_LEFT = 0;
  private static final short LIMIT_SWITCH_RIGHT = 1;

  // private float angleWhiteServo;
  private float angleRedServo;
  private boolean increasingRedServo;
  private boolean increasingWhiteServo;
  private short currentChannel;
  private float lsmSpeed;

  private final Servo whiteServo;
  private final Servo redServo;
  // The One ;)
  private final SparkMax neoMotor;
  private final Solenoid[] solenoids;
  private final PowerDistribution powerDistribution;
  private final Timer blinkingLightTimer;
  private final HttpCamera limelight;
  private final WPI_TalonSRX limitSwitchMotor;
  private final DigitalInput leftLimit;
  private final DigitalInput rightLimit;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public Robot() {
    redServo = new Servo(RED_SERVO_CHANNEL);
    whiteServo = new Servo(WHITE_SERVO_CHANNEL);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(30);
    neoMotor = new SparkMax(NEO_CHANNEL, MotorType.kBrushless);
    neoMotor.configure(config, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    limitSwitchMotor = new WPI_TalonSRX(LS_MOTOR_CHANNEL);
    rightLimit = new DigitalInput(LIMIT_SWITCH_RIGHT);
    leftLimit = new DigitalInput(LIMIT_SWITCH_LEFT);

    powerDistribution = new PowerDistribution(POWER_DISTRIBUTION_CHANNEL, ModuleType.kRev);
    blinkingLightTimer = new Timer();
    limelight = new HttpCamera("limelight", "http://10.te.am.11:5800/stream.mjpg",
        HttpCamera.HttpCameraKind.kMJPGStreamer);
    limelight.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
    CameraServer.startAutomaticCapture(limelight);

    whiteServo.set(0);
    redServo.set(180.0);
    increasingWhiteServo = true;
    increasingRedServo = true;
    currentChannel = 0;
    lsmSpeed = (float) 0.15;

    final PneumaticHub pneumaticHub = new PneumaticHub(PNEUMATIC_HUB_CHANNEL);
    solenoids = new Solenoid[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
      solenoids[i] = pneumaticHub.makeSolenoid(i);
    }
    pneumaticHub.close();

    blinkingLightTimer.start();
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
        whiteServo.getAngle());
    SmartDashboard.putNumber("Debug Red Motor Angle",
        redServo.getAngle());
    SmartDashboard.putNumber("Debug Current Channel", currentChannel);
    SmartDashboard.putNumber("Total Power via Hub",
        powerDistribution.getTotalPower());
    SmartDashboard.putNumber("Total Current via Hub",
        powerDistribution.getTotalCurrent());
    SmartDashboard.putNumber("Voltage via Hub", powerDistribution.getVoltage());

    runBlackMotor();
    runWhiteServo();
    runRedServo();
    runLightBlink();
    runLimitSwitchMotor();
    // You run the swerve drive stuff.
    // CommandScheduler.getInstance().run();
  }

  // -----------------------------------------------------------------------------------------------
  private void runBlackMotor() {
    final int MAX_VEL = 11_120 / 8;
    final double increaseFactor = 0.001;

    final double speed = neoMotor.get();
    final double velocity = neoMotor.getEncoder().getVelocity();
    final double normalizedVelocity = Math.abs(velocity / MAX_VEL);
    final double motorTemp = neoMotor.getMotorTemperature();
    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Velocity", velocity);
    SmartDashboard.putNumber("Normalized Velocity", normalizedVelocity);
    SmartDashboard.putNumber("Motor Temperature", motorTemp);

    if (!super.isEnabled())
      neoMotor.set(normalizedVelocity);

    if (normalizedVelocity < 1 && super.isEnabled())
      neoMotor.set(speed + increaseFactor);
  }

  // -----------------------------------------------------------------------------------------------
  private void runWhiteServo() {
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

    whiteServo.setAngle(currentAngle);
  }

  // -----------------------------------------------------------------------------------------------
  private void runRedServo() {
    final float maxAngle = 180;
    if (increasingRedServo) {
      angleRedServo += 5.0;
      if (angleRedServo >= maxAngle) {
        increasingRedServo = false;
      }
    } else {
      angleRedServo -= 5.0;
      if (angleRedServo <= 0.0) {
        increasingRedServo = true;
      }
    }

    redServo.setAngle(angleRedServo);
  }

  // -----------------------------------------------------------------------------------------------
  void runLightBlink() {
    if (blinkingLightTimer.hasElapsed(1.5)) {
      if (currentChannel == NUM_CHANNELS) {
        for (Solenoid solenoid : solenoids) {
          solenoid.set(false);
        }
        currentChannel = 0;
        blinkingLightTimer.reset();
        return; // Skip the rest so that we can have blank time.
      }
      solenoids[currentChannel].set(true);
      currentChannel = (short) (currentChannel + 1);
      blinkingLightTimer.reset();
    }
  }

  // -----------------------------------------------------------------------------------------------
  private void runLimitSwitchMotor() {
    SmartDashboard.putNumber("lsmSpeed 1", lsmSpeed);

    final boolean isClockWise = lsmSpeed > 0;
    limitSwitchMotor.set(ControlMode.PercentOutput, lsmSpeed);

    boolean llOnValue = leftLimit.get();
    boolean rlOnValue = rightLimit.get();
    SmartDashboard.putBoolean("Left Limit Switch status", llOnValue);
    SmartDashboard.putBoolean("Right Limit Switch status", rlOnValue);
    // Special case to get the motor in the correct section of rotation.
    if (rlOnValue && llOnValue) {
      lsmSpeed = Math.abs(lsmSpeed);
      limitSwitchMotor.set(ControlMode.PercentOutput, lsmSpeed);
      return;
    }
    if (isClockWise && rlOnValue) {
      lsmSpeed = -lsmSpeed;
      SmartDashboard.putNumber("lsmSpeed", lsmSpeed);
      limitSwitchMotor.set(ControlMode.PercentOutput, lsmSpeed);
    }
    if (!isClockWise && llOnValue) {
      lsmSpeed = -lsmSpeed;
      SmartDashboard.putNumber("lsmSpeed", lsmSpeed);
      limitSwitchMotor.set(ControlMode.PercentOutput, lsmSpeed);
    }
  }
}
