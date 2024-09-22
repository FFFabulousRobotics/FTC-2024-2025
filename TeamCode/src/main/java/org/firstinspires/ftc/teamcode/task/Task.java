package org.firstinspires.ftc.teamcode.task;

public interface Task {
    int TICK_MS = 50;

    /**
     * This method is called in the main loop of OpMode
     * It should contain the code of EACH TICK, instead of a loop in it!
     */
    void iterate();

    /**
     * To judge whether the task needs to continue.
     * @return Return true when it need to continue, otherwise false.
     */
    boolean hasNext();

    /**
     * This method is called once when the task starts looping in the TaskOpMode.
     */
    default void init() {
    }

    /**
     * The method is called when the task is finished.
     */
    default void finish() {
    }

    /**
     * The method is called when the task is canceled.
     */
    default void cancel() {
    }
}
