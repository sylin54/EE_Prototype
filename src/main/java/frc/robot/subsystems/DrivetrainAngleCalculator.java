package frc.robot.subsystems;


public class DrivetrainAngleCalculator {
    private final double trackWidthMeters = 0.14;
    
    private double lastLeftDistance = 0;
    private double lastRightDistance = 0;


    private double currentAngle = 0;

    public void update(double leftDistanceM, double rightDistanceM) {
        double leftDifference = leftDistanceM - lastLeftDistance;
        double rightDifference = rightDistanceM - lastRightDistance;

        double difference = leftDifference - rightDifference;

        double radians = difference/trackWidthMeters;


        currentAngle += radians;

        lastLeftDistance = leftDistanceM;
        lastRightDistance = rightDistanceM;


        System.out.println("difference: " + difference);
        System.out.println("radians: " + radians);
        System.out.println("total: " + getAngle());
    }

    public double getAngle() {
        return Math.toDegrees(currentAngle);
    }

    public void reset() {
        currentAngle = 0;
    }
}
