package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HookSubystem extends SubsystemBase {
    private final SparkMax motor1;
    private final SparkMax motor2;
    private final RelativeEncoder encoder;
    private boolean isRunning = false;
    private boolean speed = false;

    // Define hook positions in degrees
    public static final double POSITION_HOME = 0.0;    // Home position
    public static final double POSITION_MAX = 15.0;   // Maximum angle
    
    // Speed settings
    private static final double BASE_SPEED = 0.15;     // Lower starting speed for better control
    private static final double MAX_SPEED = 0.4;       // Lower max speed for safety
    private static final double SPEED_RAMP_RATE = 0.05; // Slower ramp rate for better control
    
    // Movement detection threshold (in degrees)
    private static final double MOVEMENT_THRESHOLD = 0.5; // Less sensitive movement detection
    
    private double currentSpeed = BASE_SPEED;
    private double lastPosition = 0.0;
    private double targetPosition = POSITION_HOME;
    private int stallCounter = 0;  // Counter for detecting stall condition

    public HookSubystem() {
        motor1 = new SparkMax(31, MotorType.kBrushless);
        motor2 = new SparkMax(32, MotorType.kBrushless);
        encoder = motor1.getEncoder();
        
        // Create configuration
        SparkMaxConfig motor1Config = new SparkMaxConfig();
        SparkMaxConfig motor2Config = new SparkMaxConfig();

        // Configure motor1 (master)
        motor1Config
            .inverted(false)
            .idleMode(IdleMode.kBrake)  // Ensure brake mode is enabled
            .smartCurrentLimit(60);
            
        motor2Config
            .follow(31, false) // Follow motor1 in same direction
            .idleMode(IdleMode.kBrake)  // Ensure brake mode is enabled
            .smartCurrentLimit(60);
            
        motor1Config.encoder
            .positionConversionFactor(1.0); // Raw encoder values for debugging
            
        motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        resetEncoder();
    }

    public void startMotors() {
        resetStallDetection();
        targetPosition = POSITION_MAX;
        currentSpeed = speed ? MAX_SPEED : BASE_SPEED;
        motor1.set(currentSpeed);
        isRunning = true;
    }

    public void startMotorInverted() {
        resetStallDetection();
        SparkMaxConfig motor2Config = new SparkMaxConfig();
        motor2Config
            .smartCurrentLimit(40)
            .inverted(false)
            .idleMode(IdleMode.kBrake);    
        SparkMaxConfig motor1Config = new SparkMaxConfig();
        motor1Config
            .smartCurrentLimit(40)
            .inverted(true)
            .idleMode(IdleMode.kBrake);   

        motor1.configure(motor1Config, null, null);
        motor2.configure(motor2Config, null, null);
        currentSpeed = BASE_SPEED;
        motor1.set(currentSpeed);
        motor2.set(currentSpeed);
        isRunning = true;
    }

    public void stopMotors() {
        motor1.set(0);
        motor2.set(0);
        isRunning = false;
        currentSpeed = BASE_SPEED;
        resetStallDetection();
    }

    private void resetStallDetection() {
        stallCounter = 0;
        lastPosition = getCurrentPosition();
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

    public void changeSpeed() {
        speed = !speed;
        if (isRunning) {
            currentSpeed = speed ? MAX_SPEED : BASE_SPEED;
            motor1.set(currentSpeed);
        }
    }

    public boolean isRunning() {
        return isRunning;
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }
    
    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - targetPosition) < 2.0;
    }
    
    public void resetEncoder() {
        encoder.setPosition(0);
        lastPosition = 0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (isRunning) {
            double currentPosition = getCurrentPosition();
            
            // Check if movement is too small
            if (Math.abs(currentPosition - lastPosition) < MOVEMENT_THRESHOLD && !isAtTarget()) {
                stallCounter++;
                
                // If stalled for more than 50 cycles (1 second), increase speed
                if (stallCounter > 50) {
                    currentSpeed = Math.min(currentSpeed + SPEED_RAMP_RATE, MAX_SPEED);
                    motor1.set(currentSpeed);
                    stallCounter = 0; // Reset counter after speed increase
                }
            } else {
                stallCounter = 0; // Reset counter if moving
            }
            
            // If we've reached the target or exceeded it, stop
            if (isAtTarget() || Math.abs(currentPosition) > POSITION_MAX) {
                stopMotors();
            }
            
            lastPosition = currentPosition;
        }
        
        // Update dashboard with detailed encoder information
        SmartDashboard.putNumber("Hook/Encoder/Raw Position", encoder.getPosition());
        SmartDashboard.putNumber("Hook/Encoder/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Hook/Target Position", targetPosition);
        SmartDashboard.putNumber("Hook/Current Speed", currentSpeed);
        SmartDashboard.putNumber("Hook/Stall Counter", stallCounter);
        SmartDashboard.putBoolean("Hook/Is At Target", isAtTarget());
        SmartDashboard.putBoolean("Hook/Is Running", isRunning);
    }
} 