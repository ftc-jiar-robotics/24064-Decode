package org.firstinspires.ftc.teamcode.decode.subsystem;
import com.bylazar.configurables.annotations.Configurable;

public abstract class Subsystem<T> {
    private boolean isLocked;

    abstract protected void set(T t);
    public final boolean set(T t, boolean isOverride) {
        if (isLocked && !isOverride) return false;

        set(t);
        return true;
    }

    abstract public T get();

    public boolean isLocked() {
        return isLocked;
    }

    public void setLocked(boolean locked) {
        isLocked = locked;
    }

    abstract public void run();
}
