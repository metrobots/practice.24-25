package frc.robot.utils;



import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.controller.PIDController;

public class PIDLib {

    private static final String KP_KEY = "kp";
    private static final String KI_KEY = "ki";
    private static final String KD_KEY = "kd";
    private static final double STORE_THRESHOLD = 0.1; // Error threshold for storing tuned parameters

    private boolean parametersStored = false;
    private double storedKp;
    private double storedKi;
    private double storedKd;
    private double maxIntegral = 1.0; // Maximum integral accumulation

    // Initialize PID controller
    private PIDController pidController;

    public double tune(double measurement, double setpoint) {
        double defaultKp = 0.00001; //DEFAULT PID PARAMETERS
        double defaultKi = 0.0;
        double defaultKd = 0.0;

        double kp = Preferences.getDouble(KP_KEY, defaultKp);
        double ki = Preferences.getDouble(KI_KEY, defaultKi);
        double kd = Preferences.getDouble(KD_KEY, defaultKd);

        // Initialize PID controller if it's null or if PID parameters change
        if (pidController == null || kp != pidController.getP() || ki != pidController.getI() || kd != pidController.getD()) {
            pidController = new PIDController(kp, ki, kd);
        }

        // Calculate error
        double error = setpoint - measurement;

        // Clamp error
        error = Math.max(Math.min(error, 1.0), -1.0);

        // Proportional tuning
        kp += 0.01 * error; // Adjust proportionally
        ki += 0.001 * error; // Adjust proportionally (smaller factor)
        kd -= 0.005 * error; // Adjust inversely proportional

        // Update PID parameters
        pidController.setPID(kp, ki, kd);

        // Clamp integral term
        double integral = pidController.getI();
        if (integral > maxIntegral) {
            pidController.setI(maxIntegral);
        } else if (integral < -maxIntegral) {
            pidController.setI(-maxIntegral);
        }

        // Check if error falls below threshold and store parameters
        if (!parametersStored && Math.abs(error) < STORE_THRESHOLD) {
            storeParameters(kp, ki, kd);
            parametersStored = true;
        }

        // Perform PID calculation
        return pidController.calculate(measurement, setpoint);
    }

    private void storeParameters(double kp, double ki, double kd) {
        // Store PID parameters to Preferences
        Preferences.setDouble(KP_KEY, kp);
        Preferences.setDouble(KI_KEY, ki);
        Preferences.setDouble(KD_KEY, kd);

        // Store parameters internally
        storedKp = kp;
        storedKi = ki;
        storedKd = kd;
    }

    public double getStoredKp() {
        return storedKp;
    }

    public double getStoredKi() {
        return storedKi;
    }

    public double getStoredKd() {
        return storedKd;
    }

    public void reset() {
        parametersStored = false;
        pidController.reset();
    }

    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = maxIntegral;
    }
}
