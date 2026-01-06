package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Channels;

public class PneumaticHubSubsystem extends SubsystemBase {
    private static final short NUM_CHANNELS = 16;

    private final PneumaticHub pneumaticHub;
    private final Solenoid[] solenoids;
    private final Timer blinkingLightTimer;
    private short currentChannel;

    public PneumaticHubSubsystem() {
        pneumaticHub = new PneumaticHub(Channels.PNEUMATIC_HUB_CHANNEL);
        currentChannel = 0;

        solenoids = new Solenoid[NUM_CHANNELS];
        for (int i = 0; i < NUM_CHANNELS; i++) {
            solenoids[i] = pneumaticHub.makeSolenoid(i);
        }

        blinkingLightTimer = new Timer();
        blinkingLightTimer.start();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Channel", currentChannel);
    }

    // ---------------------------------------------------------------------------------------------
    public Command runLightBlink() {
        return this.run(() -> {
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
        });
    }
}
