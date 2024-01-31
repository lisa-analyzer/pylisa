package it.unive.pylisa.ros;

import java.io.*;
import java.time.LocalDate;

import it.unive.ros.application.ROSApplication;
import it.unive.ros.models.rclpy.ROSNetwork;
import it.unive.ros.models.rclpy.ROSNetwork2;
import org.junit.Test;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import org.thymeleaf.TemplateEngine;
import org.thymeleaf.context.Context;
import org.thymeleaf.templatemode.TemplateMode;
import org.thymeleaf.templateresolver.ClassLoaderTemplateResolver;

public class Simple {
  @Test
  public void testOnePublisher() throws AnalysisSetupException, IOException {
    PyFrontend translator = new PyFrontend(
        "ros-tests/simple_node/one_publisher.py",
        false);
    Program program = translator.toLiSAProgram();
    LiSA lisa = new LiSA(RosTestHelpers.getLisaConf("ros-tests/simple_node/one_publisher"));
    lisa.run(program);
  }

  @Test
  public void testOnePublisherApp() throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
    new RosApplicationBuilder().withWorkDir("ros-test-outputs/simple_node/one_publisher-app")
        .withNode(new PythonROSNodeBuilder("ros-tests/simple_node/one_publisher.py"))
        .build()
        .dumpGraph();
  }

  @Test
  public void testOnePublisherRclpyApp() throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
    new RosApplicationBuilder().withWorkDir("ros-test-outputs/simple_node/one_publisher_rclpy-app")
        .withNode(new PythonROSNodeBuilder("ros-tests/simple_node/one_publisher_rclpy.py"))
        .build()
        .dumpGraph();
  }

  @Test
  public void testOnePublisherAndOneServiceRclpyApp()
      throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
    new RosApplicationBuilder().withWorkDir("ros-test-outputs/simple_node/one_publisher_one_service_rclpy-app")
        .withNode(new PythonROSNodeBuilder("ros-tests/simple_node/one_publisher_one_service_rclpy.py"))
        .build()
        .dumpGraph();
  }

  @Test
  public void testActionClient()
      throws ROSApplicationBuildException, ROSNodeBuildException, Exception {
    ROSApplication ra =  new RosApplicationBuilder().withWorkDir("ros-test-outputs/simple_node/action-app")
        .withNode(new PythonROSNodeBuilder("ros-tests/simple_node/action.py"))
        .build();
    ra.dumpGraph();
    Runtime rt = Runtime.getRuntime();
    ROSNetwork n = ra.getRosNetwork();
    ra.getRosNetwork().processEvents();
    //ra.getRosNetwork().getNetworkEntityContainer("'ros-tests/simple_node/action.py':27:16").getProcessedEvents();
    rt.exec("dot  ros-test-outputs/simple_node/action-app/graph/graph.dot -Tsvg -o ros-test-outputs/simple_node/action-app/graph/graph.svg");
    var resolver = new ClassLoaderTemplateResolver();
    resolver.setTemplateMode(TemplateMode.HTML);
    resolver.setCharacterEncoding("UTF-8");
    resolver.setPrefix("/templates/");
    resolver.setSuffix(".html");

    var context = new Context();
    context.setVariable("date", LocalDate.now().toString());
    context.setVariable("projectName", "simple_node");
    context.setVariable("graph", ra.getROSComputationalGraph());
    context.setVariable("rosNetwork", ra.getRosNetwork());
    context.setVariable("svgPath", "/Users/giacomozanatta/Projects/pylisa-ros/pylisa/ros-test-outputs/simple_node/action-app/graph/graph.svg");
    var templateEngine = new TemplateEngine();
    templateEngine.setTemplateResolver(resolver);

    var result = templateEngine.process("index", context);
    FileWriter output = new FileWriter("ros-test-outputs/simple_node/action-app/report.html");
    BufferedWriter Bout = new BufferedWriter(output);
    Bout.write(result);
    Bout.flush();
    Bout.close();
    output.close();
   // System.out.println(result);

    //Process pr = rt.exec("code ros-test-outputs/simple_node/action-app/graph/graph.dot");

    rt.exec("open ros-test-outputs/simple_node/action-app/report.html");
  }

  @Test
  public void testNewFeatures() throws Exception {
    ROSApplication ra =  new RosApplicationBuilder().withWorkDir("ros-test-outputs/new_feature/new_feature")
            .withNode(new PythonROSNodeBuilder("ros-tests/test_publish/node01.py"))
            .withNode(new PythonROSNodeBuilder("ros-tests/test_publish/node02.py"))
            .build();
    ra.getRosNetwork().processEvents();
    ra.dumpResults();
  }
}