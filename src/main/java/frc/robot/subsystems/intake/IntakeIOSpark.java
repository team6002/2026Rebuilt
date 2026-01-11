package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeIOSpark implements IntakeIO {
    private final SparkMax groundIntakeMotor;
    private final RelativeEncoder groundIntakeEncoder;
    private final SparkClosedLoopController groundIntakeController;
    private final SparkLimitSwitch leftLimit;
    private final SparkLimitSwitch rightLimit;

    private double groundIntakeReference;
    private ControlType groundIntakeType;

    public IntakeIOSpark() {
        // initialize motor
        groundIntakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);

        // initialize PID controller
        groundIntakeController = groundIntakeMotor.getClosedLoopController();

        // initalize encoder
        groundIntakeEncoder = groundIntakeMotor.getEncoder();

        // apply config
        groundIntakeMotor.configure(
                IntakeConfig.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftLimit = groundIntakeMotor.getForwardLimitSwitch();
        rightLimit = groundIntakeMotor.getReverseLimitSwitch();
        // reset target speed in init
        groundIntakeReference = 0;
        groundIntakeType = ControlType.kVoltage;
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeReference = getReference();
        inputs.intakeCurrent = getCurrent();
        inputs.intakeVoltage = getVoltage();
        inputs.intakeVelocity = getVelocity();
    }

    @Override
    public double getVelocity() {
        return groundIntakeEncoder.getVelocity();
    }

    @Override
    public double getCurrent() {
        return groundIntakeMotor.getOutputCurrent();
    }

    @Override
    public double getVoltage() {
        return groundIntakeMotor.getBusVoltage() * groundIntakeMotor.getAppliedOutput();
    }

    @Override
    public double getReference() {
        return groundIntakeReference;
    }

    @Override
    public void setVoltage(double voltage) {
        groundIntakeReference = voltage;
        groundIntakeType = ControlType.kVoltage;
    }

    @Override
    public void setReference(double velocity) {
        groundIntakeReference = velocity;
        groundIntakeType = ControlType.kVelocity;
    }

    @Override
    public boolean hasLeftCoral() {
        return leftLimit.isPressed();
    }

    @Override
    public boolean hasRightCoral() {
        return rightLimit.isPressed();
    }

    @Override
    public void PID() {
        groundIntakeController.setReference(groundIntakeReference, groundIntakeType);
    }
}
