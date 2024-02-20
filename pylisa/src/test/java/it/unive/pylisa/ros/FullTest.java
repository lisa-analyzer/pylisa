package it.unive.pylisa.ros;


import it.unive.pylisa.PyFrontend;
import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import it.unive.testing.ParsingRes;
import org.junit.Test;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class FullTest {

    static String MAIN_FOLDER = "/Users/giacomozanatta/Projects/git-repos-downloader/";
    @Test
    public void testSingleFile() throws Exception {
        //String fileName = MAIN_FOLDER + "repos" + name;
        RosApplicationBuilder rob = new RosApplicationBuilder()
                .withNode(new PythonROSNodeBuilder(
                        "/Users/giacomozanatta/Projects/git-repos-downloader/repos/chapter6/crane_plus_commander/crane_plus_commander/commander1.py"))
                .withWorkDir("test-ros-outputs/single-file-rosapp/crane_plus_commander");
        ROSApplication rosapp = rob.build();
        rosapp.getRosNetwork().processEvents();
        rosapp.dumpResults();
    }

    @Test
    public void testROSApplication() {
        int count = 0;
        int errors = 0;
        int ok = 0;
        // 1. parse file

        try {
            Scanner scanner = new Scanner(new File(MAIN_FOLDER + "/pythonFiles-new-filtered.txt"));
            FileWriter output = new FileWriter("results-19022024-new-filtered.csv");
            BufferedWriter Bout = new BufferedWriter(output);
            Bout.write("file_name,status,error_message,stack_trace");
            Bout.newLine();
            String project = "";
            RosApplicationBuilder rob = new RosApplicationBuilder();
            while (scanner.hasNextLine()) {
                ParsingRes res;
                String fileName = MAIN_FOLDER + scanner.nextLine();
                count++;
                String projName = fileName.split("/")[6];
                try {
                    if (!project.equals(projName)) {
                        if (!project.equals("")) {
                            rob.withWorkDir("ros-test-outputs/analysis-full-19022024-2/" + project);
                            project = projName;
                            ROSApplication ra = rob.build();
                            ra.getRosNetwork().processEvents();
                            ra.dumpResults();
                            ra = null;
                            rob = new RosApplicationBuilder();
                        }
                    }
                    project = projName;
                    System.out.println("Parsing " + fileName + "...");

                    // PyFrontend pyF = new PyFrontend(fileName, false);
                    // Program program = pyF.toLiSAProgram();
                    rob.withNode(new PythonROSNodeBuilder(fileName));
                    ok++;
                    res = new ParsingRes(fileName, "OK", "", "");
                } catch (Exception e) {
                    if (e instanceof ROSApplicationBuildException) {
                        res = new ParsingRes(projName, "ROS_APPLICATION_BUILDER_EXCEPTION", e.getMessage(),
                                e.getStackTrace().length > 0 ? e.getStackTrace()[0].toString() : "#none");
                    } else {
                        res = new ParsingRes(fileName, "ERROR", e.getMessage(),
                                e.getStackTrace().length > 0 ? e.getStackTrace()[0].toString() : "#none");
                        errors++;
                    }
                    rob = new RosApplicationBuilder();
                    rob.withNode(new PythonROSNodeBuilder(fileName));
                    System.out.println("[ERROR] " + e.toString());
                    project = projName;
                }
                System.out.println("Analyzed files: " + count);
                System.out.println("Errors: " + errors);
                System.out.println("Passes: " + ok);
                System.out.println("% Errors: " + ((double) errors / (double) count) * 100);
                System.out.println("% Passes: " + ((double) ok / (double) count) * 100);
                System.out.println("-----------------------------");
                Bout.write("\"" + res.fileName + "\",\"" + res.status + "\",\"" + res.error + "\",\"" + res.stackTrace
                        + "\"");
                Bout.newLine();
                Bout.flush();
            }
            rob.withWorkDir("ros-test-outputs/analysis-full-19022024-2/" + project);
            ROSApplication ra = rob.build();

            ra.dumpResults();

            ra = null;
            Bout.flush();
            Bout.close();
            scanner.close();

        } catch (Exception e) {
            System.out.println(e);
        }
    }
    @Test
    public void testPyFrontend() throws IOException {
        int count = 0;
        int errors = 0;
        int ok = 0;
        // 1. parse file

        Scanner scanner = new Scanner(new File(MAIN_FOLDER + "/pythonFiles-new.txt"));
        FileWriter output = new FileWriter("results-parser-19022024.csv");
        BufferedWriter Bout = new BufferedWriter(output);
        Bout.write("file_name,status,error_message,stack_trace");
        Bout.newLine();
        while (scanner.hasNextLine()) {
            ParsingRes res;
            String fileName = MAIN_FOLDER + scanner.nextLine();
            count++;
            System.out.println("Parsing " + fileName + "...");
            try {
                // PyFrontend pyF = new PyFrontend(fileName, false);
                // Program program = pyF.toLiSAProgram();
                new PyFrontend(fileName, false).toLiSAProgram();
                ok++;
                res = new ParsingRes(fileName, "OK", "", "");
            } catch (Exception e) {
                res = new ParsingRes(fileName, "ERROR", e.getMessage(),
                        e.getStackTrace().length > 0 ? e.getStackTrace()[0].toString() : "#none");
                errors++;
                System.out.println("[ERROR] " + e.toString());
            }
            System.out.println("Analyzed files: " + count);
            System.out.println("Errors: " + errors);
            System.out.println("Passes: " + ok);
            System.out.println("% Errors: " + ((double) errors / (double) count) * 100);
            System.out.println("% Passes: " + ((double) ok / (double) count) * 100);
            System.out.println("-----------------------------");
            Bout.write(
                    "\"" + res.fileName + "\",\"" + res.status + "\",\"" + res.error + "\",\"" + res.stackTrace + "\"");
            Bout.newLine();
            Bout.flush();
        }
        Bout.flush();
        Bout.close();
        scanner.close();
    }

    @Test
    public void testSingleRosApplication() throws ROSApplicationBuildException, Exception {
        RosApplicationBuilder rob = new RosApplicationBuilder();
        try {
            rob.withNode(new PythonROSNodeBuilder(
                    "/Users/giacomozanatta/Projects/git-repos-downloader/repos/mechaship/mechaship_teleop/mechaship_teleop/mechaship_teleop_keyboard.py"));

        } catch (ROSNodeBuildException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        rob.withWorkDir("ros-test-outputs/single-ros-app/mechashit");

        ROSApplication ra = rob.build();
        ra.dumpGraph();
    }


}
