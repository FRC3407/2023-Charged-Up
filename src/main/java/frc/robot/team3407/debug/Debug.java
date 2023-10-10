package frc.robot.team3407.debug;

/* Credit: https://github.com/HenryEwald
--> https://github.com/FRC3407/2022-Rapid-React/commit/6814744c0b36944bb1a5f6674513dd4b79e58d6a */

import java.time.format.DateTimeFormatter;
import java.time.LocalDateTime;
import java.io.FileOutputStream;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.FileWriter;
import java.io.File;


public class Debug {

    static File debugFile = null;
    static File consoleFile = null;
    static FileOutputStream outStream = null;
    static ExtPrintStream stream = null;
    static ExtPrintStream errStream = null;

    public static enum Level {
        INFO (0, "[INFO]"),
        WARNING (1, "[WARNING]"),
        ERROR (2, "[ERROR]");
    
        public final int val;
        public final String message_id;
        private Level(int v, String mi) 
        { 
            this.val = v;
            this.message_id = mi;
        }
    }


    /**
     * @return File: debugFile currently in use
     */ 
    public static File getFile()
    {
        return debugFile;
    }
    /**
     * @return boolean: true if there is a debugFile present
     */
    public static boolean hasFile()
    {
        return debugFile != null;
    }

    /**
     * Sets the system stream to both the console and the output log file
     * @throws Exception handled locally
     */
    static private void setStream()
    {
        try 
        {
            //sets System.out and File as output streams
            outStream = new FileOutputStream(consoleFile);
            stream = new ExtPrintStream(outStream, System.out, false);
            errStream = new ExtPrintStream(outStream, System.err, true);
            System.setOut(stream);
            System.setErr(errStream);
        } 
        catch (Exception e) 
        {
            Debug.log("An error ocoured", Level.ERROR);
            e.printStackTrace();
        }
    }
    /**
     * Creates a new log file based on the system time that can store the console output and manual Debug.log() commands
     * @return boolean: true if no issues prevent the file from being created
     * @throws IOExcepetion handled locally
     */
    public static boolean newLog(String flocation, String f_id)
    {
        DateTimeFormatter dtfFile = DateTimeFormatter.ofPattern("[yyyy-MM-dd_HH-mm-ss]");
        LocalDateTime now = LocalDateTime.now();  

        try
        {
            debugFile = new File(System.getProperty("user.dir") + "\\resources\\logs\\" + dtfFile.format(now) + f_id + ".txt");
            consoleFile = new File(System.getProperty("user.dir") + "\\resources\\logs\\" + dtfFile.format(now) + f_id + ".txt");
            setStream();
            debugFile.createNewFile();
        }
        catch (IOException e)
        {
            System.err.println("Failed to create Debug log or console stream");
            e.printStackTrace();
            return false;
        }
        Debug.log("Created console stream");
        System.out.println("Created debug log");
        return true;
    }

    /**
     * Logs to the current debugLog with a timestamp and marked as [Info]
     * @return boolean: true if file is successfully logged to
     * @throws IOException handled locally
     */
    public static boolean log()
    {
        return log("", Level.INFO);
    }

     /**
     * Logs to the current debugLog with a timestamp and marked as [Info]
     * @param msg - String message sent to the log
     * @return boolean: true if file is successfully logged to
     * @throws IOException handled locally
     */
    public static boolean log(String msg)
    {
        return log(msg, Level.INFO);
    }

    /**
     * Logs to the current debugLog with a timestamp and marked with a info stamp
     * @param msg - String message sent to the log
     * @param errorLevel - {@link Level} enum specifying intended log level
     * @return boolean: true if file is successfully logged to
     * @throws IOException handled locally
     */
    public static boolean log(String msg, Level errorLevel)
    {
        if(!hasFile())
        {
            return false;
        }
        StackTraceElement event_element = Thread.currentThread().getStackTrace()[2];
        FileWriter writer = null;
        BufferedWriter bufferedWriter = null;
        PrintWriter printWriter = null;
        try
        {
            try
            {
                writer = new FileWriter(debugFile, true);
            }
            catch(IOException e)
            {
                System.err.println("Failed to created FileWriter");
                e.printStackTrace();
                return false;
            }
            bufferedWriter = new BufferedWriter(writer);
            printWriter = new PrintWriter(bufferedWriter);
            printWriter.println(
                "[" + DateTimeFormatter.ofPattern("[HH:mm:ss]").format(LocalDateTime.now()) + "] "
                    + errorLevel.message_id
                    + " (" + event_element.getClassName() + "." + event_element.getMethodName() + ") : "
                    + msg
            );
            printWriter.flush();
        }
        finally
        {
            try
            {
                printWriter.close();
                bufferedWriter.close();
                writer.close();
            }
            catch (IOException e)
            {
                System.err.println("Failed to close printWriter, bufferedWriter, or writer");
                e.printStackTrace();
                return false;
            }
        }
        return true;
    }

    /**
     * Not implemented
     * @return currently zero
     */
    public static float inputDelay()
    {
        //takes time input stamps from controls and calculates the difference, printing it to the log
        return 0.f;
    }
}