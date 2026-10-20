package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class MotorLogUtil {
    public static void logMotor(TalonFX motor, String subsystem, String motorName) {
        Logger.recordOutput(subsystem + "/" + motorName + "/IsConnected", motor.isConnected());
        Logger.recordOutput(subsystem + "/" + motorName + "/StatorCurrentAmps", motor.getStatorCurrent().getValue().in(Amps));
        Logger.recordOutput(subsystem + "/" + motorName + "/SupplyCurrentAmps", motor.getSupplyCurrent().getValue().in(Amps));
        Logger.recordOutput(subsystem + "/" + motorName + "/TempFahrenheit", motor.getDeviceTemp().getValue().in(Fahrenheit));
    }
}
