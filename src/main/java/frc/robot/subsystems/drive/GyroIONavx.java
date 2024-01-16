package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

/** IO implementation for NavX */
public class GyroIONavx implements GyroIO {
  private static AHRS gyro = new AHRS(SPI.Port.kMXP);
  public double yaw;
  public double yawVelocity;

  public GyroIONavx() {
    gyro.enableLogging(true);
    gyro.zeroYaw();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = update();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity);
  }

  public boolean update() {
    yaw = gyro.getYaw();
    yawVelocity = gyro.getRate();
    return true;
  }

  public static Rotation2d getGyroscopeRotation() {
    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
    // the angle increase.
    return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
  }
}
