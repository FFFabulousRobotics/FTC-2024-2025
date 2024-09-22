package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.task.Task;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public abstract class TaskOpMode extends LinearOpMode {

    protected Map<Integer, Task> tasks;
    protected Map<Integer, Integer> taskLinks;

    public TaskOpMode() {
        tasks = new ArrayMap<>();
        taskLinks = new ArrayMap<>();
    }

    public abstract void linearInit();

    public void linearStart() {
    }

    public abstract void linearLoop();

    public void linearStop() {
    }

    @Override
    public void runOpMode() {
        linearInit();
        waitForStart();
        linearStart();
        while (opModeIsActive()) {
            for (Map.Entry<Integer, Task> taskEntry : tasks.entrySet()) {
                int taskId = taskEntry.getKey();
                Task task = taskEntry.getValue();
                if (!task.hasNext()) {
                    task.finish();
                    tasks.remove(taskId, task);
                    taskLinks.remove(taskId);
                    taskLinks.entrySet().stream()
                            .filter(e -> e.getValue() == taskId)
                            .forEach(e -> {
                                Task nextTask = tasks.get(e.getKey());
                                if (nextTask != null) nextTask.init();
                            });
                    continue;
                }
                Integer linkedTaskId = taskLinks.get(taskId);
                if (tasks.containsKey(linkedTaskId)) continue;
                task.iterate();
            }
            linearLoop();
            sleep(Task.TICK_MS);
        }
        linearStop();
    }

    public int registerTask(Task task) {
        task.init();
        return registerTask(task, -1);
    }

    /**
     * Register a task in the task map. All the tasks in the map will be iterated in
     * the main loop when it's the right time to do.
     *
     * @param task         the task that need registering.
     * @param linkedTaskId the taskId of the linked task. The task being registered will
     *                     be run after the linked task is finished. If nothing provided,
     *                     the registered task will start to run immediately.
     * @return the taskId of the registered task.
     */
    public int registerTask(Task task, int linkedTaskId) {
        int taskId = findMinFreeTaskId();
        tasks.put(taskId, task);
        taskLinks.put(taskId, linkedTaskId);
        return taskId;
    }

    /**
     * Cancel the task and all the linked-after tasks if necessary.
     *
     * @param taskId the taskId of the task needs canceling.
     */
    public void cancelTask(int taskId) {
        Task task = tasks.get(taskId);
        if (task == null) return;
        task.cancel();
        tasks.remove(taskId, task);
        taskLinks.remove(taskId);
        taskLinks.entrySet().stream()
                .filter(e -> e.getValue() == taskId)
                .forEach(e -> cancelTask(e.getKey()));
    }

    protected int findMinFreeTaskId() {
        List<Integer> taskIdList = tasks.keySet().stream().sorted().collect(Collectors.toList());
        int maxTaskId = taskIdList.get(taskIdList.size() - 1);
        for (int i = 0; i <= maxTaskId; i++) {
            if (taskIdList.contains(i)) return 1;
        }
        return maxTaskId + 1;
    }
}
