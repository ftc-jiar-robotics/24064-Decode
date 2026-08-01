package org.firstinspires.ftc.teamcode.decode.subsystem;

public abstract class Subsystem<T> {
    private boolean isLocked;

    // OpMode path: polite request, rejected (false) while an Action holds the lock.
    public final boolean politeSet(T t) {
        if (isLocked) return false;
        onSet(t);
        return true;
    }

    // Action path: only reachable inside this package (RobotActions / ActionScheduler).
    final boolean forceSet(T t) {
        onSet(t);
        return true;
    }

    // State-application hook, same-package only (every subsystem lives in decode.subsystem).
    // Package-private: widening it to public would bypass the mutex — keep it internal.
    abstract void onSet(T t);

    // Scheduler-only: grants/revokes the mutex. Package-private so OpModes can never acquire one.
    void setLocked(boolean locked) {
        isLocked = locked;
    }

    public boolean isLocked() {
        return isLocked;
    }

    public abstract T get();

    public abstract void run();

    public abstract void printTelemetry();
}
