package frc.robot.team3407.commandbased;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;


public final class Commands {

    /**
     * A base command that instantly finishes and can run in disabled mode
     */
    public static class InstantGlobal extends CommandBase {

        @Override public boolean isFinished() { return true; }
        @Override public boolean runsWhenDisabled() { return true; }

    }

    /**
     * Requires the passed in command to "finish" the desired amound of times before this wrapper finishes. Helpful for vision commands
     * where a short discontinuities can accidentally trigger an early end. 
     */
    public static class EnsureFinishCommand extends CommandBase {

        private final Command c;
        private final int threshold;
        private int count = 0;

        public EnsureFinishCommand(Command c, int f_thresh) {
            this.c = c;
            this.threshold = f_thresh;
        }

        @Override public void initialize() {
            this.count = 0;
            this.c.initialize();
        }
        @Override public void execute() {
            this.c.execute();
        }
        @Override public void end(boolean i) {
            this.c.end(i);
        }
        @Override public boolean isFinished() {
            if(this.c.isFinished()) {
                this.count++;
            }
            return this.count >= this.threshold;
        }
        @Override public boolean runsWhenDisabled() {
            return this.c.runsWhenDisabled();
        }

    }

    /**
     * A wrapper for Runnables such that they can be used in command groups and other specific scenarios
     */
    public static class LambdaCommand extends CommandBase {

        private final Runnable run;
        private final boolean when_disabled;
        public LambdaCommand(Runnable r) {
            this.run = r;
            this.when_disabled = true;
        }
        public LambdaCommand(Runnable r, boolean wd) {
            this.run = r;
            this.when_disabled = wd;
        }
        @Override public void initialize() { this.run.run(); }
        @Override public boolean isFinished() { return true; }
        @Override public boolean runsWhenDisabled() { return this.when_disabled; }


        public static class Continuous extends LambdaCommand {

            private final BooleanSupplier is_finished;
            public Continuous(Runnable r, BooleanSupplier f, boolean wd) {
                super(r, wd);
                this.is_finished = f;
            }
            public Continuous(Runnable r) { this(r, ()->false, true); }
            public Continuous(Runnable r, boolean wd) { this(r, ()->false, wd); }
            public Continuous(Runnable r, BooleanSupplier f) { this(r, f, true); }
            
            @Override public void initialize() {}
            @Override public void execute() { super.run.run(); }
            @Override public boolean isFinished() { return this.is_finished.getAsBoolean(); }


        }

        public static class Singular extends LambdaCommand {

            boolean hasrun = false;
            public Singular(Runnable r) {
                super(r);
            }
            public Singular(Runnable r, boolean wd) {
                super(r, wd);
            }

            @Override public void initialize() {
                if(!this.hasrun) {
                    super.initialize();
                    this.hasrun = true;
                }
            }

        }

    }

    /**
     * Runs the given function or runnable a certain number of loop cycles before exitting
     */
    public static class RunForCommand extends CommandBase {

        private final Runnable func;
        private final int loops;
        private int count = 0;
    
        public RunForCommand(Runnable f, int loops) {
            this.func = f;
            this.loops = loops;
        }
    
        @Override public void initialize() {
            this.count = 0;
        }
        @Override public void execute() {
            this.func.run();
            this.count++;
        }
        @Override public boolean isFinished() {
            return this.count <= this.loops;
        }
    
    }

    public static interface VerboseCommand {

        default void message(String message) {
            System.out.println(getClass().getSimpleName() + ": " + message);
        }
    
        default void start() {
            System.out.println(getClass().getSimpleName() + ": Running...");
        }
        default void finish(boolean i) {
            System.out.println(getClass().getSimpleName() +  (i ? ": Terminated." : ": Completed."));
        }
    
    }

}