// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.chargers.misc;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.ArrayList;
import java.util.List;

public class RepulsorFieldPlanner {
	private abstract static class Obstacle {
		double strength;
		boolean positive;
		
		public Obstacle(double strength, boolean positive) {
			this.strength = strength;
			this.positive = positive;
		}
		
		public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d target);
		
		protected double distToForceMag(double dist, double maxRange) {
			if (Math.abs(dist) > maxRange) {
				return 0;
			}
			if (MathUtil.isNear(0, dist, 1e-2)) {
				dist = 1e-2;
			}
			var forceMag = strength / (dist * dist);
			forceMag -= strength / (maxRange * maxRange);
			forceMag *= positive ? 1 : -1;
			return forceMag;
		}
	}
	
	private static class TeardropObstacle extends Obstacle {
		final Translation2d loc;
		final double primaryMaxRange;
		final double primaryRadius;
		final double tailStrength;
		final double tailLength;
		
		public TeardropObstacle(
			Translation2d loc,
			double primaryStrength,
			double primaryMaxRange,
			double primaryRadius,
			double tailStrength,
			double tailLength) {
			super(primaryStrength, true);
			this.loc = loc;
			this.primaryMaxRange = primaryMaxRange;
			this.primaryRadius = primaryRadius;
			this.tailStrength = tailStrength;
			this.tailLength = tailLength + primaryMaxRange;
		}
		
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var targetToLoc = loc.minus(target);
			var targetToLocAngle = targetToLoc.getAngle();
			var sidewaysPoint = new Translation2d(tailLength, targetToLoc.getAngle()).plus(loc);
			
			var positionToLocation = position.minus(loc);
			var positionToLocationDistance = positionToLocation.getNorm();
			Translation2d outwardsForce;
			if (positionToLocationDistance <= primaryMaxRange) {
				outwardsForce =
					new Translation2d(
						distToForceMag(
							Math.max(positionToLocationDistance - primaryRadius, 0),
							primaryMaxRange - primaryRadius),
						positionToLocation.getAngle());
			} else {
				outwardsForce = Translation2d.kZero;
			}
			
			var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
			var distanceAlongLine = positionToLine.getX();
			
			Translation2d sidewaysForce;
			var distanceScalar = distanceAlongLine / tailLength;
			if (distanceScalar >= 0 && distanceScalar <= 1) {
				var secondaryMaxRange =
					MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
				var distanceToLine = Math.abs(positionToLine.getY());
				if (distanceToLine <= secondaryMaxRange) {
					double strength;
					if (distanceAlongLine < primaryMaxRange) {
						strength = tailStrength * (distanceAlongLine / primaryMaxRange);
					} else {
						strength =
							-tailStrength * distanceAlongLine / (tailLength - primaryMaxRange)
								+ tailLength * tailStrength / (tailLength - primaryMaxRange);
					}
					strength *= 1 - distanceToLine / secondaryMaxRange;
					
					var sidewaysMag = tailStrength * strength * (secondaryMaxRange - distanceToLine);
					// flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
					var sidewaysTheta =
						target.minus(position).getAngle().minus(position.minus(sidewaysPoint).getAngle());
					sidewaysForce =
						new Translation2d(
							sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
							targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
				} else {
					sidewaysForce = Translation2d.kZero;
				}
			} else {
				sidewaysForce = Translation2d.kZero;
			}
			
			return outwardsForce.plus(sidewaysForce);
		}
	}
	
	static class HorizontalObstacle extends Obstacle {
		final double y;
		final double maxRange;
		
		public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
			super(strength, positive);
			this.y = y;
			this.maxRange = maxRange;
		}
		
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var dist = Math.abs(position.getY() - y);
			if (dist > maxRange) {
				return Translation2d.kZero;
			}
			return new Translation2d(0, distToForceMag(y - position.getY(), maxRange));
		}
	}
	
	private static class VerticalObstacle extends Obstacle {
		final double x;
		final double maxRange;
		
		public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
			super(strength, positive);
			this.x = x;
			this.maxRange = maxRange;
		}
		
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var dist = Math.abs(position.getX() - x);
			if (dist > maxRange) {
				return Translation2d.kZero;
			}
			return new Translation2d(distToForceMag(x - position.getX(), maxRange), 0);
		}
	}
	
	private static class LineObstacle extends Obstacle {
		final Translation2d startPoint;
		final Translation2d endPoint;
		final double length;
		final Rotation2d angle;
		final Rotation2d inverseAngle;
		final double maxRange;
		
		public LineObstacle(Translation2d start, Translation2d end, double strength, double maxRange) {
			super(strength, true);
			startPoint = start;
			endPoint = end;
			var delta = end.minus(start);
			length = delta.getNorm();
			angle = delta.getAngle();
			inverseAngle = angle.unaryMinus();
			this.maxRange = maxRange;
		}
		
		@Override
		public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
			var positionToLine = position.minus(startPoint).rotateBy(inverseAngle);
			if (positionToLine.getX() > 0 && positionToLine.getX() < length) {
				return new Translation2d(
					Math.copySign(distToForceMag(positionToLine.getY(), maxRange), positionToLine.getY()),
					angle.rotateBy(Rotation2d.kCCW_90deg));
			}
			Translation2d closerPoint;
			if (positionToLine.getX() <= 0) {
				closerPoint = startPoint;
			} else {
				closerPoint = endPoint;
			}
			return new Translation2d(
				distToForceMag(position.getDistance(closerPoint), maxRange),
				position.minus(closerPoint).getAngle());
		}
	}
	
	static final double FIELD_LENGTH = 17.6022;
	static final double FIELD_WIDTH = 8.1026;
	
	static final double SOURCE_X = 1.75;
	static final double SOURCE_Y = 1.25;
	static final List<Obstacle> FIELD_OBSTACLES =
		List.of(
			// Reef
			new TeardropObstacle(new Translation2d(4.495, 4), 0.9, 2.5, .83, 3.2, 2),
			new TeardropObstacle(new Translation2d(13.08, 4), 0.9, 2.5, .83, 3.2, 2),
			// Walls
			new HorizontalObstacle(0.0, 0.5, .5, true),
			new HorizontalObstacle(FIELD_WIDTH, 0.5, .5, false),
			new VerticalObstacle(0.0, 0.5, .5, true),
			new VerticalObstacle(FIELD_LENGTH, 0.5, .5, false),
			// Sources
			new LineObstacle(new Translation2d(0, SOURCE_Y), new Translation2d(SOURCE_X, 0), .5, .5),
			new LineObstacle(
				new Translation2d(0, FIELD_WIDTH - SOURCE_Y),
				new Translation2d(SOURCE_X, FIELD_WIDTH),
				.5,
				.5),
			new LineObstacle(
				new Translation2d(FIELD_LENGTH, SOURCE_Y),
				new Translation2d(FIELD_LENGTH - SOURCE_X, 0),
				.5,
				.5),
			new LineObstacle(
				new Translation2d(FIELD_LENGTH, FIELD_WIDTH - SOURCE_Y),
				new Translation2d(FIELD_LENGTH - SOURCE_X, FIELD_WIDTH),
				.5,
				.5));
	
	@AutoLogOutput private Translation2d goal = new Translation2d(1, 1);
	@AutoLogOutput private Rotation2d targetHeading = Rotation2d.kZero;
	private Translation2d currentTranslation = Translation2d.kZero;
	
	private static final int ARROWS_X = 40;
	private static final int ARROWS_Y = 20;
	
	private Translation2d lastGoal;
	private Pose2d[] arrowList;
	
	// A grid of arrows drawn in AScope
	@AutoLogOutput
	Pose2d[] getArrows() {
		if (goal.equals(lastGoal)) {
			return arrowList;
		}
		var list = new ArrayList<Pose2d>();
		for (int x = 0; x <= ARROWS_X; x++) {
			for (int y = 0; y <= ARROWS_Y; y++) {
				var translation =
					new Translation2d(x * FIELD_LENGTH / ARROWS_X, y * FIELD_WIDTH / ARROWS_Y);
				var force = getObstacleForce(translation, goal);
				if (force.getNorm() > 1e-6) {
					var rotation = force.getAngle();
					
					list.add(new Pose2d(translation, rotation));
				}
			}
		}
		lastGoal = goal;
		arrowList = list.toArray(new Pose2d[0]);
		return arrowList;
	}
	
	Translation2d getGoalForce(Translation2d curLocation, Translation2d goal) {
		var displacement = goal.minus(curLocation);
		if (displacement.getNorm() == 0) {
			return Translation2d.kZero;
		}
		var direction = displacement.getAngle();
		var mag = (1 + 1.0 / (1e-6 + displacement.getNorm()));
		return new Translation2d(mag, direction);
	}
	
	Translation2d getObstacleForce(Translation2d curLocation, Translation2d target) {
		var force = Translation2d.kZero;
		for (Obstacle obs : FIELD_OBSTACLES) {
			force = force.plus(obs.getForceAtPosition(curLocation, target));
		}
		return force;
	}
	
	Translation2d getForce(Translation2d curLocation, Translation2d target) {
		return getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
	}
	
	public void setGoal(Pose2d goal) {
		this.goal = goal.getTranslation();
		this.targetHeading = goal.getRotation();
	}
	
	public boolean atGoal(double tolerance) {
		return goal.getDistance(currentTranslation) < tolerance;
	}
	
	public SwerveSample sampleField(
		Translation2d currentTranslation, double maxSpeed, double slowdownDistance
	) {
		this.currentTranslation = currentTranslation;
		var err = currentTranslation.minus(goal);
		var netForce = getForce(currentTranslation, goal);
		
		double stepSize_m;
		if (err.getNorm() < slowdownDistance) {
			stepSize_m =
				MathUtil.interpolate(
					0, maxSpeed * 0.02, err.getNorm() / slowdownDistance);
		} else {
			stepSize_m = maxSpeed * 0.02;
		}
		var step = new Translation2d(stepSize_m, netForce.getAngle());
		return new SwerveSample(
			0.0,
			currentTranslation.getX(),
			currentTranslation.getY(),
			targetHeading.getRadians(),
			step.getX() / 0.02,
			step.getY() / 0.02,
			0.0, 0.0, 0.0, 0.0, new double[4], new double[4]
		);
	}
	
	public Translation2d[] getTrajectory(Translation2d current, double stepSize_m) {
		ArrayList<Translation2d> trajectory = new ArrayList<>();
		Translation2d robot = current;
		for (int i = 0; i < 200; i++) {
			var err = robot.minus(goal);
			if (err.getNorm() < stepSize_m * 1.5) {
				trajectory.add(goal);
				break;
			} else {
				var netForce = getForce(robot, goal);
				if (netForce.getNorm() == 0) {
					break;
				}
				var step = new Translation2d(stepSize_m, netForce.getAngle());
				var intermediateGoal = robot.plus(step);
				trajectory.add(intermediateGoal);
				robot = intermediateGoal;
			}
		}
		return trajectory.toArray(new Translation2d[0]);
	}
}