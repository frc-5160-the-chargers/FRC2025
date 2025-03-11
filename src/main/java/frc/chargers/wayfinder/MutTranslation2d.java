//package frc.chargers.wayfinder;
//
//import static edu.wpi.first.units.Units.Meters;
//
//import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.Vector;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.numbers.N2;
//import edu.wpi.first.units.measure.Distance;
//
//public class MutTranslation2d extends Translation2d {
//	double x = 0.0;
//	double y = 0.0;
//
//	public MutTranslation2d() {
//		super(0.0, 0.0);
//	}
//
//	@Override
//	public double getX() {
//		return x;
//	}
//
//	@Override
//	public double getY() {
//		return y;
//	}
//
//	@Override
//	public double getDistance(Translation2d other) {
//		return Math.hypot(other.getX() - x, other.getY() - y);
//	}
//
//	@Override
//	public Distance getMeasureX() {
//		return Meters.of(x);
//	}
//
//	@Override
//	public Distance getMeasureY() {
//		return Meters.of(y);
//	}
//
//	@Override
//	public Vector<N2> toVector() {
//		return VecBuilder.fill(x, y);
//	}
//
//	@Override
//	public double getNorm() {
//		return Math.hypot(x, y);
//	}
//
//	@Override
//	public Rotation2d getAngle() {
//		return new Rotation2d(x, y);
//	}
//
//	public double getRadians() {
//		double magnitude = Math.hypot(x, y);
//		if (magnitude < 1E-9) {
//			return 0.0;
//		}
//		return Math.atan2(y / magnitude, x / magnitude);
//	}
//
//	@Override
//	public Translation2d rotateBy(Rotation2d other) {
//		return new Translation2d(
//			x * other.getCos() - y * other.getSin(), x * other.getSin() + y * other.getCos());
//	}
//
//	@Override
//	public Translation2d rotateAround(Translation2d other, Rotation2d rot) {
//		return new Translation2d(
//			(x - other.getX()) * rot.getCos() - (y - other.getY()) * rot.getSin() + other.getX(),
//			(x - other.getX()) * rot.getSin() + (y - other.getY()) * rot.getCos() + other.getY());
//	}
//
//	@Override
//	public Translation2d plus(Translation2d other) {
//		return new Translation2d(x + other.getX(), y + other.getY());
//	}
//
//	@Override
//	public Translation2d minus(Translation2d other) {
//		return new Translation2d(x - other.getX(), y - other.getY());
//	}
//
//	@Override
//	public Translation2d unaryMinus() {
//		return new Translation2d(-x, -y);
//	}
//
//	@Override
//	public Translation2d times(double scalar) {
//		return new Translation2d(x * scalar, y * scalar);
//	}
//
//	@Override
//	public Translation2d div(double scalar) {
//		return new Translation2d(x / scalar, y / scalar);
//	}
//
//	public void setX(double x) {
//		this.x = x;
//	}
//
//	public void setY(double y) {
//		this.y = y;
//	}
//
//	public void set(Translation2d other) {
//		x = other.getX();
//		y = other.getY();
//	}
//
//	public void set(double x, double y) {
//		this.x = x;
//		this.y = y;
//	}
//
//	public void setPolar(double distance, double radians) {
//		x = distance * Math.cos(radians);
//		y = distance * Math.sin(radians);
//	}
//
//	public void plusMut(Translation2d other) {
//		x += other.getX();
//		y += other.getY();
//	}
//
//	public void minusMut(Translation2d other) {
//		x -= other.getX();
//		y -= other.getY();
//	}
//
//	public void timesMut(double scalar) {
//		x *= scalar;
//		y *= scalar;
//	}
//
//	public void divMut(double scalar) {
//		x /= scalar;
//		y /= scalar;
//	}
//
//	public void rotateByMut(Rotation2d other) {
//		double xNew = x * other.getCos() - y * other.getSin();
//		double yNew = x * other.getSin() + y * other.getCos();
//		x = xNew;
//		y = yNew;
//	}
//
//	public void rotateAroundMut(Translation2d other, Rotation2d rot) {
//		double xNew =
//			(x - other.getX()) * rot.getCos() - (y - other.getY()) * rot.getSin() + other.getX();
//		double yNew =
//			(x - other.getX()) * rot.getSin() + (y - other.getY()) * rot.getCos() + other.getY();
//		x = xNew;
//		y = yNew;
//	}
//
//	@Override
//	public boolean equals(Object obj) {
//		return obj instanceof Translation2d other
//			       && Math.abs(other.getX() - x) < 1E-9
//			       && Math.abs(other.getY() - y) < 1E-9;
//	}
//}
