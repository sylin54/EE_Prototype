package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainFeedForwardCalculator {

    private final ProfiledPIDController profiledPIDController;
    private final SimpleMotorFeedforward feedForward;

    private final LinearFilter averageFilter = LinearFilter.movingAverage(20);

    public DrivetrainFeedForwardCalculator(
            double kP,
            double kI,
            double kD,
            double kA,
            double kV,
            double kS,
            double maxSpeed,
            double maxAcceleration) {

        profiledPIDController = new ProfiledPIDController(
                kP,
                kI,
                kD,
                new TrapezoidProfile.Constraints(
                        maxSpeed,
                        maxAcceleration
                )
        );

        feedForward = new SimpleMotorFeedforward(
                kS,
                kV,
                kA
        );
    }

    public double calculate(double currentVelocity, double wantedVelocity) {
        double feedForwardOutput = feedForward.calculate(wantedVelocity);
        double pidOutput = profiledPIDController.calculate(currentVelocity, wantedVelocity);
        double output = feedForwardOutput + pidOutput;

        return output;
    }

    public double calculate(double currentVelocity, double wantedVelocity, String loggingPrefix) {
        double feedForwardOutput = feedForward.calculate(wantedVelocity);
        double pidOutput = profiledPIDController.calculate(currentVelocity, wantedVelocity);
        double output = feedForwardOutput + pidOutput;

        SmartDashboard.putNumber(loggingPrefix + " current velocity", currentVelocity);
        SmartDashboard.putNumber(loggingPrefix + " target velocity", wantedVelocity);
        SmartDashboard.putNumber(loggingPrefix + " current velocity average", averageFilter.calculate(currentVelocity));
        SmartDashboard.putNumber(loggingPrefix + " feed forward output", feedForwardOutput);
        SmartDashboard.putNumber(loggingPrefix + " pid output ", pidOutput);
        SmartDashboard.putNumber(loggingPrefix + " total output", output);

        return output;
    }
}