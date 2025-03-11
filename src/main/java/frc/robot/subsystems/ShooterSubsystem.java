package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // Motor controllers
    private final SparkMax commonMotor;
    private final SparkMax algaeMotor;
    private final SparkMax coralMotor;
    private final SparkMax rotateMotor;
    
    // PID controller for position control
    private final SparkPIDController rotationPIDController;
    
    // Encoder for position feedback
    private final SparkEncoder rotationEncoder;
    
    // Constants for position control
    private static final double MAX_POSITION_OFFSET = 100.0;   // Maximum allowed position offset
    private static final double MIN_POSITION_OFFSET = -100.0;  // Minimum allowed position offset
    
    // PID constants for position control
    private static final double kP = 0.1;  // Proportional gain
    private static final double kI = 0.0;  // Integral gain
    private static final double kD = 0.0;  // Derivative gain
    private static final double kIz = 0.0; // I-zone
    private static final double kFF = 0.0; // Feed forward
    
    // Control constants
    private static final double MANUAL_CONTROL_SPEED = 0.3;  // Speed for manual control (0-1)
    private static final double POSITION_TOLERANCE = 0.5;    // Position tolerance in encoder units
    
    // Current target position
    private double targetPosition = 0.0;
    
    // Motor running state
    private boolean isRunning = false;
    
    // Control mode
    private enum ControlMode {
        POSITION,  // Closed-loop position control
        MANUAL     // Manual control (for initial setup)
    }
    
    private ControlMode currentMode = ControlMode.POSITION;
    
    /**
     * Creates a new ShooterSubsystem with position control.
     */
    public ShooterSubsystem() {
        // Initialize motors with their CAN IDs
        commonMotor = new SparkMax(43, MotorType.kBrushless);
        algaeMotor = new SparkMax(44, MotorType.kBrushless);
        coralMotor = new SparkMax(45, MotorType.kBrushless);
        rotateMotor = new SparkMax(41, MotorType.kBrushless);
        
        // Get the encoder and PID controller
        rotationEncoder = rotateMotor.getEncoder();
        rotationPIDController = rotateMotor.getPIDController();
        
        // Configure the rotation motor
        SparkMaxConfig rotateConfig = new SparkMaxConfig();
        rotateConfig
            .smartCurrentLimit(30)  // Limit current to prevent damage
            .inverted(false)        // Set direction
            .idleMode(IdleMode.kBrake);  // Use brake mode to help hold position
        
        rotateMotor.configure(rotateConfig, null, null);
        
        // Configure PID controller
        if (rotationPIDController != null) {
            // Set PID coefficients
            rotationPIDController.setP(kP);
            rotationPIDController.setI(kI);
            rotationPIDController.setD(kD);
            rotationPIDController.setIZone(kIz);
            rotationPIDController.setFF(kFF);
            
            // Set output range for the PID controller
            rotationPIDController.setOutputRange(-1.0, 1.0);
        } else {
            System.err.println("WARNING: PID controller not found on rotate motor!");
        }
        
        // We don't reset the encoder position to zero
        // Instead, we use the current position as our reference point
        if (rotationEncoder != null) {
            // Get the current position and set it as our target
            targetPosition = rotationEncoder.getPosition();
            System.out.println("Initial shooter position: " + targetPosition);
        } else {
            System.err.println("WARNING: Encoder not found on rotate motor!");
        }
    }
    
    /**
     * Start the coral motors
     */
    public void startCoralMotors() {
        SparkMaxConfig commonConfig = new SparkMaxConfig();
        commonConfig
            .smartCurrentLimit(40)
            .inverted(false);    
        SparkMaxConfig coralConfig = new SparkMaxConfig();
        coralConfig
            .smartCurrentLimit(40)
            .follow(0, true);
        
        // Set motor2 to run in opposite direction
        coralMotor.configure(coralConfig, null, null);
        commonMotor.configure(commonConfig, null, null);
        
        isRunning = true;
    }

    /**
     * Start the algae motors
     */
    public void startAlgaeMotors() {
        SparkMaxConfig commonConfig = new SparkMaxConfig();
        commonConfig
            .smartCurrentLimit(40)
            .inverted(false);    
        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig
            .smartCurrentLimit(40)
            .follow(0, true);
        
        // Set motor2 to run in opposite direction
        algaeMotor.configure(algaeConfig, null, null);
        commonMotor.configure(commonConfig, null, null);
        
        isRunning = true;
    }
    
    /**
     * Set a specific target position for the shooter.
     * 
     * @param position The target position in encoder units
     */
    public void setTargetPosition(double position) {
        if (rotationPIDController == null) {
            return;
        }
        
        // Clamp position to valid range based on initial position
        double currentPosition = getCurrentPosition();
        double initialPosition = currentPosition - (targetPosition - currentPosition);
        double minPosition = initialPosition + MIN_POSITION_OFFSET;
        double maxPosition = initialPosition + MAX_POSITION_OFFSET;
        
        targetPosition = MathUtil.clamp(position, minPosition, maxPosition);
        
        // Set the target position
        rotationPIDController.setReference(targetPosition, SparkPIDController.ControlType.kPosition);
        
        // Ensure we're in position control mode
        currentMode = ControlMode.POSITION;
    }
    
    /**
     * Get the current position of the shooter.
     * 
     * @return The current position in encoder units
     */
    public double getCurrentPosition() {
        if (rotationEncoder != null) {
            return rotationEncoder.getPosition();
        }
        return 0.0;
    }
    
    /**
     * Get the current target position.
     * 
     * @return The target position in encoder units
     */
    public double getTargetPosition() {
        return targetPosition;
    }
    
    /**
     * Check if the shooter is at the target position.
     * 
     * @return True if at target position, false otherwise
     */
    public boolean isAtTargetPosition() {
        if (rotationEncoder == null) {
            return false;
        }
        
        double currentPosition = rotationEncoder.getPosition();
        return Math.abs(currentPosition - targetPosition) < POSITION_TOLERANCE;
    }
    
    /**
     * Rotate the shooter up (positive direction) at a fixed speed.
     */
    public void rotateUp() {
        currentMode = ControlMode.MANUAL;
        
        // Check if we're at the maximum position limit
        if (rotationEncoder != null) {
            double currentPosition = rotationEncoder.getPosition();
            double initialPosition = currentPosition - (targetPosition - currentPosition);
            double maxPosition = initialPosition + MAX_POSITION_OFFSET;
            
            if (currentPosition >= maxPosition) {
                stopRotation();
                return;
            }
        }
        
        // Set motor to rotate up at the specified speed
        rotateMotor.set(MANUAL_CONTROL_SPEED);
    }
    
    /**
     * Rotate the shooter down (negative direction) at a fixed speed.
     */
    public void rotateDown() {
        currentMode = ControlMode.MANUAL;
        
        // Check if we're at the minimum position limit
        if (rotationEncoder != null) {
            double currentPosition = rotationEncoder.getPosition();
            double initialPosition = currentPosition - (targetPosition - currentPosition);
            double minPosition = initialPosition + MIN_POSITION_OFFSET;
            
            if (currentPosition <= minPosition) {
                stopRotation();
                return;
            }
        }
        
        // Set motor to rotate down at the specified speed
        rotateMotor.set(-MANUAL_CONTROL_SPEED);
    }
    
    /**
     * Stop the rotation motor.
     * In position mode, this will maintain the current position.
     * In manual mode, this will stop the motor.
     */
    public void stopRotation() {
        if (currentMode == ControlMode.MANUAL) {
            rotateMotor.set(0);
            
            // Update target position to current position
            if (rotationEncoder != null) {
                targetPosition = rotationEncoder.getPosition();
            }
        }
        
        // Switch to position control mode
        currentMode = ControlMode.POSITION;
        
        // Set the PID controller to maintain the current position
        if (rotationPIDController != null && rotationEncoder != null) {
            rotationPIDController.setReference(targetPosition, SparkPIDController.ControlType.kPosition);
        }
    }
    
    /**
     * Stop all motors.
     */
    public void stopMotors() {
        commonMotor.set(0);
        algaeMotor.set(0);
        coralMotor.set(0);
        isRunning = false;
    }
    
    /**
     * Toggle the coral motors.
     */
    public void toggleCoralMotors() {
        if (isRunning) {
            stopMotors();
        } else {
            startCoralMotors();
        }
    }
    
    /**
     * Toggle the algae motors.
     */
    public void toggleAlgaeMotors() {
        if (isRunning) {
            stopMotors();
        } else {
            startAlgaeMotors();
        }
    }
    
    /**
     * Check if the motors are running.
     * 
     * @return True if running, false otherwise
     */
    public boolean isRunning() {
        return isRunning;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        // Safety check for position limits
        if (rotationEncoder != null) {
            double currentPosition = rotationEncoder.getPosition();
            double initialPosition = currentPosition - (targetPosition - currentPosition);
            double minPosition = initialPosition + MIN_POSITION_OFFSET;
            double maxPosition = initialPosition + MAX_POSITION_OFFSET;
            
            // If we're outside the limits, stop and reset the target position
            if (currentPosition > maxPosition) {
                rotateMotor.set(0);
                targetPosition = maxPosition;
                
                if (rotationPIDController != null) {
                    rotationPIDController.setReference(targetPosition, SparkPIDController.ControlType.kPosition);
                }
                
                System.out.println("Reached maximum position limit");
            } else if (currentPosition < minPosition) {
                rotateMotor.set(0);
                targetPosition = minPosition;
                
                if (rotationPIDController != null) {
                    rotationPIDController.setReference(targetPosition, SparkPIDController.ControlType.kPosition);
                }
                
                System.out.println("Reached minimum position limit");
            }
        }
    }
} 