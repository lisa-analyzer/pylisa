
package it.unive.ros;

import it.unive.ros.application.PythonROSNodeBuilder;
import it.unive.ros.application.ROSApplication;
import it.unive.ros.application.RosApplicationBuilder;
import it.unive.ros.application.exceptions.ROSApplicationBuildException;
import it.unive.ros.application.exceptions.ROSNodeBuildException;

public class Main {

    public static ROSApplication tb4() throws ROSNodeBuildException, ROSApplicationBuildException {
        return new RosApplicationBuilder()
                .withNode(new PythonROSNodeBuilder("ros-tests/turtlegoal/tb4_publish_goal.py"))
                .withNode(new PythonROSNodeBuilder("ros-tests/turtlegoal/tb4_planner.py"))
                .withNode(new PythonROSNodeBuilder("ros-tests/turtlegoal/extended_kalman_filter.py"))
                .withNode(new PythonROSNodeBuilder("ros-tests/turtlegoal/tb4_undock.py"))
                .withWorkDir("OUTPUTS/tb4")
                .build();
    }

    public static ROSApplication ROS2OSMCartrographer() throws ROSNodeBuildException, ROSApplicationBuildException {
        return new RosApplicationBuilder()
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/git-repos/ros2_osm_cartographer/route_network/src/route_network/nodes/plan_route.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/git-repos/ros2_osm_cartographer/route_network/src/route_network/nodes/route_network.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/git-repos/ros2_osm_cartographer/route_network/src/route_network/nodes/rviz_goal.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/git-repos/ros2_osm_cartographer/route_network/src/route_network/nodes/viz_plan.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/git-repos/ros2_osm_cartographer/route_network/src/route_network/nodes/viz_routes.py"))
                // add also nodes in osm_cartography
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/git-repos/ros2_osm_cartographer/osm_cartography/src/osm_cartography/nodes/viz_osm.py"))
                .withNode(new PythonROSNodeBuilder(
                        "ros-tests/git-repos/ros2_osm_cartographer/osm_cartography/src/osm_cartography/nodes/osm_server.py"))
                // .withNode(new
                // PythonROSNodeBuilder("ros-tests/git-repos/ros2_osm_cartographer/osm_cartography/src/osm_cartography/nodes/osm_client.py"))
                .withWorkDir("OUTPUTS/ros2_osm_cartographer")
                .build();
    }

    public static ROSApplication TurtlePong() throws ROSNodeBuildException, ROSApplicationBuildException {
        return new RosApplicationBuilder()
                .withNode(new PythonROSNodeBuilder("ros-tests/git-repos/Turtle_pong/src/ping_pong/ping_pong/ball.py"))
                // .withNode(new
                // PythonROSNodeBuilder("ros-tests/git-repos/Turtle_pong/src/ping_pong/ping_pong/key_teleop.py"))
                // FAIL
                .withNode(
                        new PythonROSNodeBuilder("ros-tests/git-repos/Turtle_pong/src/ping_pong/ping_pong/paddles.py"))
                .withWorkDir("OUTPUTS/Turtle_Pong")
                .build();
    }

    public static ROSApplication AutoCarROS2() throws ROSNodeBuildException, ROSApplicationBuildException {
        return new RosApplicationBuilder()
                .withNode(new PythonROSNodeBuilder("ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/clickplanner.py"))
                .withNode(
                        new PythonROSNodeBuilder("ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/globalplanner.py"))
                .withNode(new PythonROSNodeBuilder("ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/localisation.py"))
                .withNode(new PythonROSNodeBuilder("ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/localplanner.py"))
                .withNode(new PythonROSNodeBuilder("ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/tracker.py"))
                .withWorkDir("OUTPUTS/AutoCarROS2")
                .build();
    }

    public static void main(String[] args) throws Exception {
        ROSApplication ra = new RosApplicationBuilder().withNode(new PythonROSNodeBuilder("ros-tests/minimal.py"))
                .withWorkDir("OUTPUT")
                .build();
        ra.dumpGraph();
        // .withNode(new PythonROSNodeBuilder("ros-tests/minimal.py"))

        // TB4
        /*
         * .withNode(new
         * PythonROSNodeBuilder("ros-tests/turtlegoal/tb4_publish_goal.py"))
         * .withNode(new PythonROSNodeBuilder("ros-tests/turtlegoal/tb4_planner.py"))
         * .withNode(new
         * PythonROSNodeBuilder("ros-tests/turtlegoal/extended_kalman_filter.py"))
         * .withNode(new PythonROSNodeBuilder("ros-tests/turtlegoal/tb4_undock.py"))
         */
        // AutoCarROS2
        /*
         * .withNode(new PythonROSNodeBuilder(
         * "ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/clickplanner.py"))
         * .withNode(new PythonROSNodeBuilder(
         * "ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/globalplanner.py"))
         * .withNode(new PythonROSNodeBuilder(
         * "ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/localisation.py"))
         * .withNode(new PythonROSNodeBuilder(
         * "ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/localplanner.py"))
         * .withNode(new PythonROSNodeBuilder(
         * "ros-tests/git-repos/AutoCarROS2/autocar_nav/nodes/tracker.py"))
         */
        // .withNode(new PythonROSNodeBuilder("ros-tests/main2.py"))
        // .withNode(new PythonROSNodeBuilder("ros-tests/minimal3.py"))
        // .withNode(new PythonROSNodeBuilder("ros-tests/main2.py"))
        // .withPermissions("INPUT/permissions/minimal_publisher/permissions.xml")
        // .withPermissions("INPUT/permissions/minimal_publisher2/permissions.xml")
        // .withPermissions("INPUT/permissions/node_4/permissions.xml")
        // .withPermissions("INPUT/permissions/minimal_publisher/permissions.xml")
        // .withPermissions("OUTPUT/permissions/minimal_publisher2/permissions.xml")
        // .withPermissions("OUTPUT/permissions/minimal_subscriber/permissions.xml")
        // .withPermissions("INPUT/permissions/minimal_publisher/permissions.xml")
        // .withPermissions("OUTPUT/permissions/node_2/permissions.xml")
        // .withPermissions("OUTPUT/permissions/node_3/permissions.xml")
        // .withPermissions("OUTPUT/permissions/node_4/permissions.xml")
        // .withPermissions("OUTPUT/permissions/node_1/permissions.xml")
        // .withPermissions("INPUT/permissions/minimal_publisher/permissions.xml")
        // .withNode(new PythonROSNodeBuilder("ros-tests/main_test.py"))
        // .withWorkDir("OUTPUT")
        // .build();
        ROSApplication rosApp = tb4();
        rosApp.dumpPermissions();
        rosApp.dumpGraph();
        rosApp.dumpSecureGraph();
        rosApp.dumpInfo();
        rosApp.dumpGraphUndAdjMatrix();
        rosApp.dumpGraphDirAdjMatrix();

        rosApp = AutoCarROS2();
        rosApp.dumpPermissions();
        rosApp.dumpGraph();
        rosApp.dumpSecureGraph();
        rosApp.dumpInfo();
        rosApp.dumpGraphUndAdjMatrix();
        rosApp.dumpGraphDirAdjMatrix();

        rosApp = ROS2OSMCartrographer();
        rosApp.dumpPermissions();
        rosApp.dumpGraph();
        rosApp.dumpSecureGraph();
        rosApp.dumpInfo();
        rosApp.dumpGraphUndAdjMatrix();
        rosApp.dumpGraphDirAdjMatrix();

        /*
         * rosApp = TurtlePong();
         * rosApp.dumpPermissions();
         * rosApp.dumpGraph();
         * rosApp.dumpSecureGraph();
         * rosApp.dumpInfo();
         * rosApp.dumpGraphUndAdjMatrix();
         * rosApp.dumpGraphDirAdjMatrix();
         */
    }
}