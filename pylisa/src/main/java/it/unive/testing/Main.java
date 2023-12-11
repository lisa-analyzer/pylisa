package it.unive.testing;

import it.unive.pylisa.PyFrontend;
import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class Main {

	static String MAIN_FOLDER = "/Users/giacomozanatta/Projects/git-repos-downloader/";

	public static void testROSApplication() {
		int count = 0;
		int errors = 0;
		int ok = 0;
		// 1. parse file

		try {
			Scanner scanner = new Scanner(new File(MAIN_FOLDER + "/pythonFiles.txt"));
			FileWriter output = new FileWriter("results.csv");
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
							rob.withWorkDir("analysis-new-2/" + project);
							project = projName;
							ROSApplication ra = rob.build();
							ra.dumpPermissions();
							ra.dumpGraph();
							ra.dumpSecureGraph();
							ra.dumpInfo();
							ra.dumpGraphUndAdjMatrix();
							ra.dumpGraphDirAdjMatrix();
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
						rob = new RosApplicationBuilder();
					} else {
						res = new ParsingRes(fileName, "ERROR", e.getMessage(),
								e.getStackTrace().length > 0 ? e.getStackTrace()[0].toString() : "#none");
						errors++;
					}

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
			Bout.flush();
			Bout.close();
			scanner.close();

		} catch (Exception e) {
			System.out.println(e);
		}
	}

	public static void testPyFrontend() throws IOException {
		int count = 0;
		int errors = 0;
		int ok = 0;
		// 1. parse file

		Scanner scanner = new Scanner(new File(MAIN_FOLDER + "/pythonFiles.txt"));
		FileWriter output = new FileWriter("results-parser.csv");
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

	public static void testSingleRosApplication() throws ROSApplicationBuildException, Exception {
		RosApplicationBuilder rob = new RosApplicationBuilder();
		try {
			rob.withNode(
					new PythonROSNodeBuilder("/Users/giacomozanatta/Projects/pylisa-ros/pylisa/ros-tests/minimal.py"));
		} catch (ROSNodeBuildException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		rob.withWorkDir("analysis-single");
		rob.build().dumpGraph();
	}

	public static void main(String[] args) throws ROSApplicationBuildException, Exception {
		testSingleRosApplication();
		// testROSApplication();
		// testPyFrontend();
	}

}
