#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mpt_ros/msg/motion_plan_request.hpp"

#include <Eigen/Dense>
#include <sstream>
#include <fstream>
#include <cstdlib>

using std::placeholders::_1;
using std::shared_ptr;

std::vector<unsigned char> loadFile(const std::string& fileName) {
    std::string home = "/home/root"; //getenv("HOME");
    std::string resourceDir = home + "/fog_ws/src/fogros2/mpt_ros/resources/ompl-app/3D/";
    std::vector<unsigned char> data;
    std::ifstream file(resourceDir + fileName, std::ios::binary);
    file.unsetf(std::ios::skipws); // don't skip newlines
    file.seekg(0, std::ios::end); // seek to end to get file size
    data.reserve(file.tellg()); // reserve bytes for file
    file.seekg(0, std::ios::beg); // seek to beginning to start read.
    data.insert(data.begin(), std::istream_iterator<unsigned char>(file),
                std::istream_iterator<unsigned char>());
    return data;
}

template <typename T>
void sendMesh(const shared_ptr<rclcpp::Publisher<T>> &pub, const std::string& file) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = loadFile(file);
    msg.layout.data_offset = 0;
    msg.layout.dim.emplace_back();
    msg.layout.dim[0].label = "dae";
    msg.layout.dim[0].size = msg.data.size();
    msg.layout.dim[0].stride = 1;
    pub->publish(msg);
}

//rclcpp::Publisher<PubMsg>*
template <typename T, typename Derived>
void sendMatrix(const shared_ptr<rclcpp::Publisher<T>> &pub, const Eigen::MatrixBase<Derived>& m) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(m.rows() * m.size());
    for (int c=0 ; c<m.cols() ; ++c)
        for (int r=0 ; r<m.rows() ; ++r)
            msg.data.push_back(m(r, c));
    msg.layout.data_offset = 0;
    msg.layout.dim.emplace_back();
    msg.layout.dim[0].size = m.rows();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim.emplace_back();
    msg.layout.dim[1].size = m.cols();
    msg.layout.dim[1].stride = 1;
    pub.publish(msg);
}

static bool gotPlan_{false};

static void motionPlanCallback(const std::shared_ptr<rclcpp::Node> &node, const std_msgs::msg::Float64MultiArray &msg) {
    RCLCPP_INFO(node->get_logger(), "Got Motion Plan");
    if (msg.layout.dim.size() != 2) {
        RCLCPP_INFO(node->get_logger(), "Motion planning failed");
    } else {
        RCLCPP_INFO(node->get_logger(), "Motion plan has %u waypoints, %zu bytes",
                 msg.layout.dim[0].size,
                 msg.data.size() * sizeof(msg.data[0]));

        unsigned int rows = msg.layout.dim[0].size;
        unsigned int cols = msg.layout.dim[1].size;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m(rows, cols);

        for (unsigned i=0 ; i<rows ; ++i)
            for (unsigned j=0 ; j<cols ; ++j)
                m(i,j) = msg.data[i * msg.layout.dim[0].stride + j];

        std::ostringstream str;
        str << m;
        RCLCPP_INFO(node->get_logger(), "Waypoints: \n%s", str.str().c_str());
    }
    gotPlan_ = true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("mpt_client");

    // std::string envFile;
    // std::string robotFile;

    // n.getParam("env", envFile);
    // n.getParam("robot", robotFile);
    RCLCPP_INFO(node->get_logger(), "Initing");

    auto envMeshPub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("environment_mesh", 1000);
    auto robotMeshPub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("robot_mesh", 1000);
    auto motionPlanRequestPub = node->create_publisher<mpt_ros::msg::MotionPlanRequest>("motion_plan_request", 1000);
    // rclcpp::Publisher boundsPub = n.advertise<std_msgs::msg::Float64MultiArray>("environment_bounds", 1000);
    // rclcpp::Publisher planPub = n.advertise<std_msgs::msg::Float64MultiArray>("plan_start_to_goal", 1000);
    auto motionPlanSub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "motion_plan", 1000,
      [node] (const std_msgs::msg::Float64MultiArray msg){
          motionPlanCallback(node, msg);
        });
    //   std::bind(&motionPlanCallback, node , std::placeholders::_1));
    rclcpp::Rate loop_rate(10);
    RCLCPP_INFO(node->get_logger(), "Entering loop");

    while (rclcpp::ok()) {
        // rclcpp::spin(node); //Fix hang problem
        if (node->count_subscribers("environment_mesh") > 0 &&
            node->count_subscribers("robot_mesh") > 0 &&
            node->count_subscribers("motion_plan_request") > 0)
            // boundsPub.getNumSubscribers() > 0 &&
            // planPub.getNumSubscribers() > 0)
            break;
        loop_rate.sleep();
    }

    if (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Sending env");
        mpt_ros::msg::MotionPlanRequest req;
        Eigen::Matrix<double, 3, 2> bounds;
        Eigen::Matrix<double, 7, 2> startAndGoal;

        req.max_planning_time = 600.0;
        req.min_planning_time = 0;
        req.min_solution_cost = std::numeric_limits<double>::infinity();

        req.start_names = std::vector<std::string>(
            {{ "tx","ty","tz", "angle", "axis.x", "axis.y", "axis.z" }});
        req.goal_names = req.start_names;
        req.bounds_names = std::vector<std::string>(
            {{ "tx", "ty", "tz" }});

        std::string scenario = "Apartment"; //Apartment, Home, cubicles, Twistycool, ;
        if (scenario == "Apartment") {
            sendMesh(envMeshPub, "Apartment_env.dae");
            sendMesh(robotMeshPub, "Apartment_robot.dae");
            bounds <<
                -73.76, 295.77,
                -179.59, 168.26,
                -0.03, 90.39;
            startAndGoal <<
                241.81,   -31.19,
                106.15,   -99.85,
                36.46,    36.46,
                3.12413936107, 3.12413936107,
                0.0, 0.0,
                0.0, 0.0,
                -1.0, -1.0;
            // We target a cost of 650, which can take from 20 s to
            // over 10 min on 1 core.
            req.min_solution_cost = 650;
        } else if (scenario == "Home") {
            sendMesh(envMeshPub, "Home_env.dae");
            sendMesh(robotMeshPub, "Home_robot.dae");
            bounds <<
                -383.802642822, 324.997131348,
                -371.469055176, 337.893371582,
                -0.196851730347, 142.332290649;
            startAndGoal <<
                252.95, 262.95,
                -214.95, 75.05,
                46.19, 46.19,
                0, 0,
                1, 1,
                0, 0,
                0, 0;
            req.min_solution_cost = 2500;
        } else if (scenario == "cubicles") {
            sendMesh(envMeshPub, "cubicles_env.dae");
            sendMesh(robotMeshPub, "cubicles_robot.dae");
            bounds <<
          -508.88, 319.62,
          -230.13, 531.87,
          -123.75, 101.0;
            startAndGoal <<
          -4.96, 200,
          -40.62, -40.62,
          70.57, 70.57,
                0, 0,
                1, 1,
                0, 0,
                0, 0;
            req.min_solution_cost = 2800;
    } else if (scenario == "Twistycool") {
            // Ug, this seems to have a solution that is just to
            // interpolate from start to goal.
            sendMesh(envMeshPub, "Twistycool_env.dae");
            sendMesh(robotMeshPub, "Twistycool_robot.dae");
            bounds <<
          53.46, 402.96,
          -21.25, 269.25,
          -476.86, -91;
            startAndGoal <<
          270, 270,
          160,160,
          -200, -400,
                0, 0,
                1, 1,
                0, 0,
                0, 0;
    } else {
            throw std::invalid_argument("unknown scenario: " + scenario);
        }
        
        using Clock = std::chrono::steady_clock;

        auto startTime = Clock::now();
        for (int i=0 ; i<3 ; ++i)
            req.bounds_min.push_back(bounds(i, 0));
        for (int i=0 ; i<3 ; ++i)
            req.bounds_max.push_back(bounds(i, 1));
        for (int i=0 ; i<7 ; ++i)
            req.start_config.push_back(startAndGoal(i, 0));
        for (int i=0 ; i<7 ; ++i)
            req.goal_config.push_back(startAndGoal(i, 1));

        motionPlanRequestPub->publish(req);
        //sendMatrix(boundsPub, bounds);
        //sendMatrix(planPub, startAndGoal);

        // int count = 0;
        // while (rclcpp::ok()) {
        //     std_msgs::msg::String msg;

        //     std::stringstream str;
        //     str << "Hello! " << count;
        //     msg.data = str.str();

        //     RCLCPP_INFO(node->get_logger(), "%s", msg.data.c_str());

        //     chatter_pub.publish(msg);

        //     rclcpp::spin(node);

        //     loop_rate.sleep();
        //     ++count;
        // }
        while (rclcpp::ok() && !gotPlan_) {
            // rclcpp::spin(node);
            loop_rate.sleep();
        }

        auto elapsed = Clock::now() - startTime;

        std::cout << "Elapsed time: " << std::chrono::duration<double>(elapsed).count()
                  << std::endl;
    }

    return 0;
}
