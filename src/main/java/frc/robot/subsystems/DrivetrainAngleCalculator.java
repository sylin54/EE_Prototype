package frc.robot.subsystems;


public class DrivetrainAngleCalculator {
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

        lastLeftDistance = leftDistanceM;
        lastRightDistance = rightDistanceM;


        System.out.println("difference: " + difference);
        System.out.println("degrees: " + degrees);

        return currentAngle;
    }

    public double getAngle() {
        return currentAngle;
    }

    public void reset(double angle) {
        currentAngle = angle;
    }
}
