package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeConfig {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    static {
        intakeConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kBrake)
                .inverted(IntakeConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        intakeConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        intakeConfig
                .closedLoop
                .pidf(
                        IntakeConstants.kP,
                        IntakeConstants.kI,
                        IntakeConstants.kD,
                        IntakeConstants.kFF)
                .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        intakeConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
    }
}
