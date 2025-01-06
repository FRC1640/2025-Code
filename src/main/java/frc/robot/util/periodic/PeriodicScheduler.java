package frc.robot.util.periodic;

import java.util.HashSet;
import java.util.Set;

public class PeriodicScheduler {
    private Set<PeriodicBase> periodics = new HashSet<>();

    private PeriodicScheduler() {}

    private static PeriodicScheduler instance;
    public static PeriodicScheduler getInstance() {
        if (instance == null) {
            instance = new PeriodicScheduler();
        }
        return instance;
    }
    
    public void addPeriodic(PeriodicBase periodic) {
        this.periodics.add(periodic);
    }

    public void run() {
        periodics.forEach(PeriodicBase::periodic);
    }
}
