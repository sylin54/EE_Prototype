package frc.robot.subsystems;


public class DrivetrainOdometryCalculator {
    private final double trackWidthMeters = 0.014;
    
    private double lastLeftDistance = 0;
    private double lastRightDistance = 0;


    private double currentAngle = 0;

    public double update(double leftDistanceM, double rightDistanceM) {
        double leftDifference = leftDistanceM - lastLeftDistance;
        double rightDifference = rightDistanceM - lastRightDistance;

        double difference = leftDifference - rightDifference;

        double degrees = Math.tan(difference/trackWidthMeters);

        currentAngle += degrees;

        return currentAngle;
    }
}
