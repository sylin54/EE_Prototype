package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainAngleCalculator {
    private final double trackWidthMeters = 0.141;
    
    private double lastLeftDistance = 0;
    private double lastRightDistance = 0;

    private Rotation2d currentAngle =  new Rotation2d();

    public void update(double leftDistanceM, double rightDistanceM) {
        double leftDifference = leftDistanceM - lastLeftDistance;
        double rightDifference = rightDistanceM - lastRightDistance;

        double difference = rightDifference - leftDifference;

        double radians = difference/trackWidthMeters;


        currentAngle = currentAngle.plus(new Rotation2d(radians));

        lastLeftDistance = leftDistanceM;
        lastRightDistance = rightDistanceM;
    }

    public Rotation2d getAngle() {
        return currentAngle;
    }

    public void reset() {
        currentAngle = new Rotation2d();
    }
}
