// package frc.robot.subsystems;

// // import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// // import com.ctre.phoenix.motorcontrol.NeutralMode;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.LEDPattern;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// // import com.ctre.phoenix.sensors.CANCoderConfiguration;
// // import com.ctre.phoenix.sensors.WPI_CANCoder;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// // import com.ctre.phoenix.motorcontrol.ControlMode;
// import edu.wpi.first.units.measure.Angle;





// public class LedSubsystem extends SubsystemBase {

//   private static final int kPwmPort = 0;
//   private static final int kLedCount = 28;

  

//   private final AddressableLED m_led;
//   private final AddressableLEDBuffer m_ledBuffer;

//   public boolean getCoral() {
//     return !ShooterSubsystem.photoSensor.get();
//   }

    
//   public LedSubsystem() {
//     m_led = new AddressableLED(kPwmPort);
//         m_ledBuffer = new AddressableLEDBuffer(kLedCount);
//         m_led.setLength(m_ledBuffer.getLength());
//         m_led.start();

       
   
//   }

//   @Override
//   public void periodic() {

//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }

//   public void updateLEDs() {
    
//     for (int i = 0; i < m_ledBuffer.getLength(); i++) {
//         if (getCoral()) {
//             // 綠色: R=0, G=255, B=0
//             m_ledBuffer.setRGB(i, 0, 255, 0);
//         } else {
//             // 紅色: R=255, G=0, B=0
//             m_ledBuffer.setRGB(i, 255, 0, 0);
//         }
//     }
   
//     m_led.setData(m_ledBuffer);
// }
 







// } 
