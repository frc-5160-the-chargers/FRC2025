// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

/** Creates a new ExampleSubsystem.  */
class ExampleSubsystem(
    motorID: Int,
    /**
     * An example of a property that is also a constructor argument.
     * The value of inputID depends on the value passed in
     * when the ExampleSubsystem is created.
     */
    private val toPrint: Int,
): SubsystemBase() {
    private val motor = PWMSparkMax(motorID)

    init {
        // This is run when the ExampleSubsystem instance is created.
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    fun exampleMethodCommand(): Command =
        // Inline construction of command goes here.
        // Subsystem.run and Subsystem.runOnce implicitly requires `this` subsystem.
        run {
            motor.setVoltage(5.0)
        }.andThen(runOnce { println(toPrint) })

    /**
     * An example of a property with a custom getter.
     * Every time this property is accessed, its value will be re-computed to a new value.
     *
     * Properties like these can be used to query a boolean state of the subsystem (for example, a digital sensor).
     */
    val motorIsAlive: Boolean get() = motor.isAlive

    /**
     * An example subsystem property using a lazy initializer.
     *
     * A lazy initializer will initialize the property the first
     * time it is accessed, rather than when the subsystem is created.
     *
     * A lazy initializer can be used to avoid expensive operations or
     * resource allocation if an object is not used. For instance,
     * giving a simulation object a lazy initializer can prevent it
     * from using unnecessary CPU or memory on the real robot.
     */
    val lazyMotorSim by lazy {
        DCMotorSim(
            DCMotor.getNEO(1),
            1.0, 0.004
        )
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}