package frc.robot;

public class Scheduler {

    // Starts an async repeating task
    void launchAsyncTask(RobotAsyncTask task) {
        new Thread() {
            @Override
            public void run() {
                task.run();
            }
        }.start();
    }

    /** Schedules a task */
    public void schedule(RobotAsyncTask task) {
        launchAsyncTask(task);
    }

    /** Task to run asynchronously */
    @FunctionalInterface
    public static interface RobotAsyncTask extends Runnable {}
    
}
