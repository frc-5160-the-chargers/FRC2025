package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.chargers.hardware.CTREUtil;
import frc.chargers.utils.CommandUtil;
import frc.chargers.utils.Tracer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.concurrent.atomic.AtomicInteger;

public class CompetitionRobot extends LoggedRobot {
    private static final boolean IS_REPLAY = false;

    public CompetitionRobot() {
        setUseTiming(false);
        if (IS_REPLAY) {
            Logger.setReplaySource(new WPILOGReader(LogFileUtil.findReplayLog()));
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.addDataReceiver(new WPILOGWriter());
        LoggedPowerDistribution.getInstance();
        Logger.start();
    }

    private static class TestSys1 extends SubsystemBase {
        public TestSys1() {
            setDefaultCommand(
                this.run(() -> System.out.println("TestSys1 Default"))
            );
        }
    }

    private static class TestSys2 extends SubsystemBase {
        public TestSys2() {
            setDefaultCommand(
                this.run(() -> System.out.println("TestSys2 Default"))
            );
        }
    }

    @Override
    public void autonomousInit() {
        var counter = new AtomicInteger();
        var s1 = new TestSys1();
        var s2 = new TestSys2();
        var cmd = CommandUtil.nonBlockingParallel(
            Commands.run(() -> System.out.println("Hi1"), s1)
                .until(() -> counter.getAndIncrement() > 100),
            Commands.run(() -> System.out.println("Hi2"), s2)
        );
        cmd.schedule();
    }

    @Override
    public void robotPeriodic() {
        Tracer.trace("Command Scheduler", CommandScheduler.getInstance()::run);
        Tracer.trace("Signal Refresh", CTREUtil::refreshSignals);
    }

    @Override
    public void loopFunc() {
        Tracer.trace("Periodic Loop", super::loopFunc);
    }
}
