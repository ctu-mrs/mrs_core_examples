/* includes //{ */

/*each ros package must have these*/
#include <rclcpp/rclcpp.hpp>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/msg/odometry.hpp>

/* custom msgs of MRS group */
#include <mrs_msgs/msg/reference_stamped.hpp>

/* for calling simple ros services */
#include <std_srvs/srv/trigger.hpp>

#include <waypoint_flier_simple/params.h>

#include <random>

using namespace std::chrono_literals;

//}


/* what is subnode for paramter loading*/
namespace example_waypoint_flier_simple
{
    class WaypointFlierSimple : public rclcpp::Node
    {
    public:
        WaypointFlierSimple(const rclcpp::NodeOptions& options);
        void intialize();
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Clock::SharedPtr clock_;

        /* ros paramters*/
        // | -------------------------- flags ------------------------- |

        /* is set to true when the nodelet is initialized, useful for rejecting callbacks that are called before the node is initialized */
        std::atomic<bool> is_initialized_ = false;
        bool loaded_successfully = true;

        /* by default, the node component is deactivated, it only starts publishing goals when activated */
        std::atomic<bool> active_ = false;

        /* by default, the nodel component is deactivated, it only starts publishing goals when activated */
        std::atomic<bool> have_odom_ = false;

        /* variables to store the coordinates of the current goal */
        double goal_x_ = 0.0;
        double goal_y_ = 0.0;
        double goal_z_ = 2.0;

        /* variables to store the maximum limit for the random waypoint generator */
        double max_x_;
        double max_y_;
        double max_z_;

        /* ROS messages which store the current reference and odometry */
        mrs_msgs::msg::ReferenceStamped ref_;
        nav_msgs::msg::Odometry         current_odom_;

        // | ---------------------- ROS subscribers --------------------- |
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        void            callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

        // | ---------------------- ROS publishers --------------------- |
        void callbackMainTimer();
        rclcpp::TimerBase::SharedPtr    timer_publisher_reference_;
        rclcpp::Publisher<mrs_msgs::msg::ReferenceStamped>::SharedPtr publisher_reference_;

        // | ---------------------- ROS timers --------------------- |

        rclcpp::TimerBase::SharedPtr timer_initializer_;
        void      initialize();

        // | ---------------------- ROS service servers --------------------- |

        bool               callbackStart([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_start_;

        // | ------------------ Additional functions ------------------ |

        double distance(const mrs_msgs::msg::ReferenceStamped& waypoint, const nav_msgs::msg::Odometry& odom);

        double getRandomDouble(double min, double max);
        
    };

    WaypointFlierSimple::WaypointFlierSimple(const rclcpp::NodeOptions& options): Node("example_waypoint_flier_simple", options)
    {
        timer_initializer_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&WaypointFlierSimple::initialize,this));
    }

    void WaypointFlierSimple::initialize(){

        node_ = this->shared_from_this();
        clock_ = node_->get_clock();
        
        /*------------ load parameters --------- */
        loaded_successfully &= utils::load_param("max_x", max_x_, 10.0, *node_);
        loaded_successfully &= utils::load_param("max_y", max_y_, 10.0, *node_);
        loaded_successfully &= utils::load_param("max_z", max_z_, 5.0, *node_);
        
        if (!loaded_successfully)
        {
            RCLCPP_INFO_ONCE(node_->get_logger(),"Failed to load non optional parameters");
        }        

        // | -------- initialize a publisher for UAV reference -------- |
        publisher_reference_ = node_->create_publisher<mrs_msgs::msg::ReferenceStamped>("~/reference_out", 1);
        timer_publisher_reference_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&WaypointFlierSimple::callbackMainTimer, this));

        // | --------- initialize a subscriber for UAV Odometry -----------|
        const std::function<void(const nav_msgs::msg::Odometry::SharedPtr)> odom_cbk = std::bind(&WaypointFlierSimple::callbackOdom, this, std::placeholders::_1);
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("~/odom_in",10,odom_cbk);

        // |--------------- service server for starting waypoint following ---------------|
        srv_server_start_ = create_service<std_srvs::srv::Trigger>("~/start_waypoint_flying", std::bind(&WaypointFlierSimple::callbackStart, this, std::placeholders::_1, std::placeholders::_2));

        is_initialized_ = true;

        timer_initializer_->cancel();

    }

    // |------------------ msg_callbacks ------------------------|
    /* callbackodom() //{ */
    void WaypointFlierSimple::callbackOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
        /* do not continue if nodelet is not initialized*/
        if (!is_initialized_)
        {
            return;
        }
        // | -------------- save the current UAV odometry ------------- |
        current_odom_ = *msg;
        have_odom_    = true;
        
    }
    /* }*/

    // |------------------ timer callbacks ---------------------------|
    /* callbackMainTimer() //{ */
    void WaypointFlierSimple::callbackMainTimer(){
        if (!active_)
        {
            RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlierSimple]: waypoint flier is not activated yet."); 
        }else
        {
            const double curr_dist = distance(ref_, current_odom_);

            if(curr_dist < 1.0){
                RCLCPP_INFO_STREAM(node_->get_logger(), "[WaypointFlierSimple]: Goal reached!");
                /* select new reference point */
                goal_x_ = getRandomDouble(-max_x_,max_x_);
                goal_y_ = getRandomDouble(-max_y_,max_y_);
                goal_z_ = getRandomDouble(2,max_z_);

                RCLCPP_INFO_STREAM(node_->get_logger(), "[WaypointFlierSimple]: New goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);
            }

            ref_.reference.position.x = goal_x_;
            ref_.reference.position.y = goal_y_;
            ref_.reference.position.z = goal_z_;  
            ref_.reference.heading = 0.0;
            
            publisher_reference_->publish(ref_);
        }
        
    }
    /* }*/

    bool WaypointFlierSimple::callbackStart([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res){

        // | ------------------- activation service ------------------- |
        // only activates the main loop when the nodelet is initialized and receiving odometry

        if (!is_initialized_) {

            res->success = false;
            res->message = "Waypoint flier not initialized!";
            RCLCPP_WARN(node_->get_logger(), "[WaypointFlierSimple]: Cannot start waypoint following, nodelet is not initialized.");
            return true;
        }

        if (!have_odom_) {

            res->success = false;
            res->message = "Waypoint flier is not receiving odometry!";
            RCLCPP_WARN(node_->get_logger(), "[WaypointFlierSimple]: Cannot start, nodelet is not receiving odometry!");
            return true;
        }

        active_ = true;

        RCLCPP_INFO(node_->get_logger(), "[WaypointFlierSimple]: Starting waypoint following.");
        RCLCPP_INFO_STREAM(node_->get_logger(), "[WaypointFlierSimple]: Goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);

        res->success = true;
        res->message = "Starting waypoint following.";

        return true;

    }

    // | -------------------- support functions ------------------- |

    /* distance() //{ */

    double WaypointFlierSimple::distance(const mrs_msgs::msg::ReferenceStamped& waypoint, const nav_msgs::msg::Odometry& odom) {

        // | ------------- distance between two 3D points ------------- |

        return sqrt((pow(waypoint.reference.position.x - odom.pose.pose.position.x, 2)) + (pow(waypoint.reference.position.y - odom.pose.pose.position.y, 2)) +
                    (pow(waypoint.reference.position.z - odom.pose.pose.position.z, 2)));
        }

    double WaypointFlierSimple::getRandomDouble(double min, double max) {

        // | --------- random double in the min and max bounds -------- |

        float r = (float)rand() / (float)RAND_MAX;
        return min + r * (max - min);
    }
    
} // end of example_waypoint_flier_simple namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(example_waypoint_flier_simple::WaypointFlierSimple)
