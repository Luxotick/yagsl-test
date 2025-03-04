package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax ElevatorMotor1;
    private final SparkMax ElevatorMotor2;
    private boolean isRunning = false;

    public ElevatorSubsystem() {
        // Initialize motors with their CAN IDs
        ElevatorMotor1 = new SparkMax(61, MotorType.kBrushless);
        ElevatorMotor2 = new SparkMax(62, MotorType.kBrushless);

        // Set initial motor configurations to prevent errors
        configureMotor(ElevatorMotor1, 40, false);
        configureMotor(ElevatorMotor2, 40, true);
    }

    private void configureMotor(SparkMax motor, int currentLimit, boolean isInverted) {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(currentLimit).inverted(isInverted);
        motor.configure(motorConfig, null, null);
    }

    private void runMotors(double speed1, double speed2) {
        ElevatorMotor1.set(speed1);
        ElevatorMotor2.set(speed2);
        isRunning = true;
    }

    public void startMotors() {
        // Start motors with normal configuration (motor1 runs forward, motor2 runs backward)
        runMotors(0.3, 0.3); // Adjust speed as needed
    }

    public void startMotorsInverted() {
        // Start motors with inverted configuration (motor1 runs backward, motor2 runs forward)
        runMotors(-0.3, -0.3); // Adjust speed as needed
    }

    public void stopMotors() {
        ElevatorMotor1.set(0);
        ElevatorMotor2.set(0);
        isRunning = false;
    }

    public void toggleMotors() {
        if (isRunning) {
            stopMotors();
        } else {
            startMotors();
        }
    }

    public void toggleMotorsReverse() {
        if (isRunning) {
            stopMotors();
        } else {
            startMotorsInverted();
        }
    }

    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
