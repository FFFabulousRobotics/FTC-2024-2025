package org.firstinspires.ftc.teamcode.task;

public class SleepTask implements Task {
    protected int remainingSleepTicks;

    public SleepTask(int sleepTicks) {
        remainingSleepTicks = sleepTicks;
    }

    @Override
    public void iterate() {
        remainingSleepTicks--;
    }

    @Override
    public boolean hasNext() {
        return remainingSleepTicks > 0;
    }
}
