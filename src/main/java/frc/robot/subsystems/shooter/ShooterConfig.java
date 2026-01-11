package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterConfig {
    public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();

    static {
        shooterConfig
                .disableFollowerMode()
                .idleMode(IdleMode.kCoast)
                .inverted(ShooterConstants.kInverted)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);
        shooterConfig.encoder.quadratureAverageDepth(2).quadratureMeasurementPeriod(10);
        shooterConfig
                .closedLoop
                .pidf(
                        ShooterConstants.kP,
                        ShooterConstants.kI,
                        ShooterConstants.kD,
                        ShooterConstants.kFF)
                .outputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterConfig.limitSwitch.forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false);
    }
}
