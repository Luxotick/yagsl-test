package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubystem extends SubsystemBase {
    private final SparkMax motor1;
    private final SparkMax motor2;
    private boolean isRunning = false;

    public HookSubystem() {
        // Initialize motors with their CAN IDs
        motor1 = new SparkMax(31, MotorType.kBrushless);
        motor2 = new SparkMax(32, MotorType.kBrushless);

        
    }

    public void startMotors() {
        SparkMaxConfig motor2Config = new SparkMaxConfig();
         motor2Config
        .smartCurrentLimit(40)
        .inverted(true);    
        SparkMaxConfig motor1Config = new SparkMaxConfig();
        motor1Config
        .smartCurrentLimit(40)
        .inverted(false);   
        // Reset motor controllers to factory defaults


        // Set motor2 to run in opposite direction
        motor2.configure(motor2Config, null, null);

        motor1.configure(motor1Config, null, null);

        motor1.set(0.3); // Run at 50% speed - adjust as needed
        motor2.set(0.3); // This will run in opposite direction due to inversion
        isRunning = true;
    }

    public void startMotorInverted() {
        SparkMaxConfig motor2InvertedConfig = new SparkMaxConfig();
        motor2InvertedConfig
       .smartCurrentLimit(40)
       .inverted(false);    
       SparkMaxConfig motor1InvertedConfig = new SparkMaxConfig();
       motor1InvertedConfig
       .smartCurrentLimit(40)
       .inverted(true);   

        motor1.configure(motor1InvertedConfig, null, null);
        motor2.configure(motor2InvertedConfig, null, null);
        motor1.set(0.03); // Run at 50% speed - adjust as needed
        motor2.set(0.03); // This will run in opposite direction due to inversion
        isRunning = true;
    }

    public void stopMotors() {
        motor1.set(0);
        motor2.set(0);
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
            startMotorInverted();
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