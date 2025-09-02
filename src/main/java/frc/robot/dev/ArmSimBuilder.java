package frc.robot.dev;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class ArmSimBuilder {
    public static class SimParams {
        private LinearSystem<N2, N1, N2> m_plant;
        private DCMotor m_gearbox;
        private double m_gearing;
        private double m_armLenMeters;
        private double m_minAngle;
        private double m_maxAngle;
        private boolean m_simulateGravity;
    }
    
    public static class NeedsGearingStage {
        private final SimParams params;
        
        private NeedsGearingStage(SimParams params) {
            this.params = params;
        }
        
        public NeedsLengthStage reduction(double reduction) {
            params.m_gearing = reduction;
            return new NeedsLengthStage(params);
        }

        public NeedsLengthStage multiplier(double multiplier) {
            params.m_gearing = 1 / multiplier;
            return new NeedsLengthStage(params);
        }
        
        public NeedsGearingStage simGravity() {
            params.m_simulateGravity = true;
            return this;
        }
    }

    public static class NeedsLengthStage {
        private final SimParams params;

        private NeedsLengthStage(SimParams params) {
            this.params = params;
        }

        public NeedsLimitsStage length(Distance armJointLength) {
            params.m_armLenMeters = armJointLength.in(Meters);
            return new NeedsLimitsStage(params);
        }

        public NeedsLengthStage simGravity() {
            params.m_simulateGravity = true;
            return this;
        }
    }
    
    public static class NeedsLimitsStage {
        private final SimParams params;

        private NeedsLimitsStage(SimParams params) {
            this.params = params;
        }

        public NeedsStartingAngleStage limits(Angle minAngle, Angle maxAngle) {
            params.m_minAngle = minAngle.in(Radians);
            params.m_maxAngle = maxAngle.in(Radians);
            return new NeedsStartingAngleStage(params);
        }

        public NeedsLimitsStage simGravity() {
            params.m_simulateGravity = true;
            return this;
        }
    }
    
    public static class NeedsStartingAngleStage {
        private final SimParams params;

        private NeedsStartingAngleStage(SimParams params) {
            this.params = params;
        }

        public SingleJointedArmSim startingAngle(Angle startingAngle) {
            return new SingleJointedArmSim(
                params.m_plant,
                params.m_gearbox,
                params.m_gearing,
                params.m_armLenMeters,
                params.m_minAngle,
                params.m_maxAngle,
                params.m_simulateGravity,
                startingAngle.in(Radians)
            );
        }

        public NeedsStartingAngleStage simGravity() {
            params.m_simulateGravity = true;
            return this;
        }
    }
}
