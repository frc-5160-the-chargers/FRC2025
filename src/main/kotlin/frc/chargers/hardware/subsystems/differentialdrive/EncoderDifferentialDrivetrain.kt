@file:Suppress("unused", "MemberVisibilityCanBePrivate", "LeakingThis")
package frc.chargers.hardware.subsystems.differentialdrive

import com.batterystaple.kmeasure.interop.average
import com.batterystaple.kmeasure.quantities.*
import com.batterystaple.kmeasure.units.degrees
import com.batterystaple.kmeasure.units.meters
import com.batterystaple.kmeasure.units.seconds
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.chargers.hardware.motorcontrol.Motor
import frc.chargers.hardware.imu.HeadingProvider
import frc.chargers.hardware.imu.ZeroableHeadingProvider
import frc.chargers.hardware.subsystems.PoseEstimatingDrivetrain
import frc.chargers.wpilibextensions.ChargerSysIdRoutine
import frc.chargers.wpilibextensions.Rotation2d
import frc.chargers.wpilibextensions.angle
import frc.chargers.wpilibextensions.kinematics.ChassisPowers
import frc.chargers.wpilibextensions.kinematics.ChassisSpeeds
import monologue.Annotations.Log
import kotlin.jvm.optionals.getOrNull

/**
 * A standard differential drive, with encoders.
 */
open class EncoderDifferentialDrivetrain(
    private val name: String,
    private val leftMotors: List<Motor>,
    private val rightMotors: List<Motor>,
    private val constants: DifferentialDriveConstants,
    private val gyro: HeadingProvider? = null
): PoseEstimatingDrivetrain() {
    /* Private implementation */
    private val wheelRadius = constants.wheelDiameter / 2

    private val leftEncoder = leftMotors[0].encoder
    private val rightEncoder = rightMotors[0].encoder

    private val kinematics = DifferentialDriveKinematics(constants.width.inUnit(meters))
    private val poseEstimator = DifferentialDrivePoseEstimator(
        kinematics,
        Rotation2d(0.0),
        0.0, 0.0,
        Pose2d()
    )

    @Log private val field = Field2d()
    private val robotWidget = field.getObject(name)

    private val leftWheelTravel: Distance get() = leftEncoder.angularPosition * wheelRadius
    private val rightWheelTravel: Distance get() = rightEncoder.angularPosition * wheelRadius
    private val calculatedHeading: Angle get() = (leftWheelTravel - rightWheelTravel) / constants.width

    /* Public API */
    init {
        if (constants.invertMotors){
            for (motor in rightMotors){
                motor.configure(inverted = !motor.inverted)
            }
        }
        for (motor in leftMotors + rightMotors){
            motor.configure(velocityPID = constants.velocityPID, currentPosition = 0.degrees)
        }
        when (constants.pathAlgorithm){
            PathAlgorithm.LTV -> {
                AutoBuilder.configureLTV(
                    { robotPose },
                    { resetPose(it) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    0.02,
                    constants.pathReplanningConfig,
                    { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }, // function used to determine if alliance flip is necessary
                    this
                )
            }

            PathAlgorithm.RAMSETE -> {
                AutoBuilder.configureRamsete(
                    { robotPose },
                    { resetPose(it) },
                    { currentSpeeds },
                    { speeds: ChassisSpeeds -> velocityDrive(speeds) },
                    constants.pathReplanningConfig,
                    { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }, // function used to determine if alliance flip is necessary,
                    this
                )
            }
        }
    }

    /**
     * The total linear distance traveled since the start of the match.
     */
    val distanceTraveled: Distance
        get() = listOf(leftWheelTravel, rightWheelTravel).average()

    /**
     * The current linear velocity of the robot.
     */
    val velocity: Velocity
        get() = listOf(leftEncoder.angularVelocity, rightEncoder.angularVelocity)
            .average() * wheelRadius

    /**
     * The current pose of the robot.
     */
    override val robotPose: Pose2d
        get() = poseEstimator.estimatedPosition

    /**
     * Resets the robot pose to a certain value.
     */
    override fun resetPose(pose: Pose2d) {
        if (gyro is ZeroableHeadingProvider){
            gyro.zeroHeading(pose.rotation.angle)
            // uses pose rotation since gyro is going to be zeroed
            poseEstimator.resetPosition(
                pose.rotation,
                leftWheelTravel.siValue,
                rightWheelTravel.siValue,
                pose
            )
        }else{
            poseEstimator.resetPosition(
                Rotation2d(gyro?.heading ?: calculatedHeading),
                leftWheelTravel.siValue,
                rightWheelTravel.siValue,
                pose
            )
        }
    }

    /**
     * Adds a vision measurement to the drivetrain's pose estimator.
     */
    override fun addVisionMeasurement(pose: Pose2d, timestamp: Time, stdDevs: Matrix<N3, N1>?) {
        if (stdDevs != null){
            poseEstimator.addVisionMeasurement(pose, timestamp.inUnit(seconds), stdDevs)
        }else{
            poseEstimator.addVisionMeasurement(pose, timestamp.inUnit(seconds))
        }
    }

    /**
     * Gets the current [ChassisSpeeds] of the robot.
     */
    val currentSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(
            DifferentialDriveWheelSpeeds(
                (leftEncoder.angularVelocity * wheelRadius).inUnit(meters / seconds),
                (rightEncoder.angularVelocity * wheelRadius).inUnit(meters / seconds),
            )
        )

    /**
     * Drives using "tank controls", a system by which each side of the drivetrain is controlled independently.
     * @param leftPower the power of the left side of the drivetrain (from [-1..1]).
     * @param rightPower the power of the right side of the drivetrain (from [-1..1]).
     */
    fun tankDrive(leftPower: Double, rightPower: Double) {
        leftMotors.forEach{ it.speed = leftPower }
        rightMotors.forEach{ it.speed = rightPower }
    }

    /**
     * Drives the robot at a certain power forward and with a certain amount of rotation.
     * @param power the power with which to drive forward (from [-1..1]).
     * @param rotation the power with which to rotate (proportional to the angular velocity, or how quickly the heading changes). (Must be from [-1..1]).
     */
    fun arcadeDrive(power: Double, rotation: Double = 0.0, squareInputs: Boolean = true){
        val wheelSpeeds = DifferentialDrive.arcadeDriveIK(power,rotation, squareInputs)
        tankDrive(wheelSpeeds.left,wheelSpeeds.right)
    }

    /**
     * Drives the robot at a certain power forward and with a certain amount of steering.
     * This method makes turning easier at high speeds.
     * @param power the power with which to drive forward (from [-1..1]).
     * @param steering the amount of steering (inversely proportional to the turn radius).
     * Changing this value is can be thought of as changing how far a car's steering wheel is turned.
     * (Must be from [-1..1]).
     */
    fun curvatureDrive(power: Double, steering: Double, allowTurnInPlace: Boolean = true){
        val wheelSpeeds = DifferentialDrive.curvatureDriveIK(power, steering, allowTurnInPlace)
        tankDrive(wheelSpeeds.left, wheelSpeeds.right)
    }

    /**
     * Stops the robot.
     */
    fun stop(){
        tankDrive(0.0,0.0)
    }

    /**
     * Calls [arcadeDrive] with a [ChassisPowers].
     */
    fun arcadeDrive(chassisPowers: ChassisPowers, squareInputs: Boolean = true) {
        arcadeDrive(power = chassisPowers.xPower, rotation = chassisPowers.rotationPower, squareInputs)
    }

    /**
     * Calls [curvatureDrive] with a [ChassisPowers].
     */
    fun curvatureDrive(chassisPowers: ChassisPowers, allowTurnInPlace: Boolean = true) {
        curvatureDrive(power = chassisPowers.xPower, steering = chassisPowers.rotationPower, allowTurnInPlace)
    }

    fun velocityDrive(xVelocity: Velocity, yVelocity: Velocity, rotationSpeed: AngularVelocity): Unit =
        velocityDrive(ChassisSpeeds(xVelocity,yVelocity,rotationSpeed))

    fun velocityDrive(speeds: ChassisSpeeds){
        val wheelSpeeds = kinematics.toWheelSpeeds(speeds)
        velocityDrive(
            wheelSpeeds.leftMetersPerSecond.ofUnit(meters/seconds),
            wheelSpeeds.rightMetersPerSecond.ofUnit(meters/seconds)
        )
    }

    fun velocityDrive(leftSpeed: Velocity, rightSpeed: Velocity) {
        leftMotors.forEach{
            it.setVelocitySetpoint(
                leftSpeed / wheelRadius,
                constants.velocityFF(leftSpeed / wheelRadius)
            )
        }
        rightMotors.forEach{
            it.setVelocitySetpoint(
                rightSpeed / wheelRadius,
                constants.velocityFF(rightSpeed / wheelRadius)
            )
        }
    }

    val sysIdRoutine = ChargerSysIdRoutine(
        this,
        setVoltage = { v ->
            leftMotors.forEach { it.voltageOut = v }
            rightMotors.forEach { it.voltageOut = v }
        }
    )

    override fun periodic(){
        log("$name/DistanceTraveled(Meters)", distanceTraveled.inUnit(meters))
        log("$name/Pose2d", robotPose)
        log("$name/ChassisSpeeds", currentSpeeds)

        if (DriverStation.isDisabled()) stop()
        poseEstimator.update(
            Rotation2d(gyro?.heading ?: calculatedHeading),
            leftWheelTravel.inUnit(meters),
            rightWheelTravel.inUnit(meters),
        )
        super.periodic()
        robotWidget.pose = poseEstimator.estimatedPosition
    }
}