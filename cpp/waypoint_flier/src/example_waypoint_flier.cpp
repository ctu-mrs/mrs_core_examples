/* includes //{*/

#include <rclcpp/rclcpp.hpp>

/* for loading dynamic parameters while ros2 component is runnning */
#include <mrs_lib/dynparam_mgr.h>
#include <string.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous manipulation by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

/* for storing information about the state of the uav( position, twist) + covariances*/
#include <nav_msgs/msg/odometry.hpp>

/* custom msgs of MRS group */
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>

/* for calling simple ros services */
#include <std_srvs/srv/trigger.hpp>

/* for operations with matrices */
#include <Eigen/Dense>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;


namespace example_waypoint_flier{

    /* class ExampleWaypointFlier //{ */

    class ExampleWaypointFlier : public rclcpp::Node
    {   
        public:
            ExampleWaypointFlier(rclcpp::NodeOptions options);
            void intialize();

        private:
            rclcpp::Node::SharedPtr node_;
            rclcpp::Clock::SharedPtr clock_;
            /* ros parameters*/
            std::string _uav_name_;
            bool is_initialized_ = false;

            // | ---------------------- msg callbacks -------------|
            mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_odometry_;
            mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;

            std::shared_ptr<mrs_lib::DynparamMgr> dynparam_mgr_;
            
            void              callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);
            std::atomic<bool> have_goal_ = false;
            std::atomic<bool> waypoint_reached_ = false;

            // | -------------------- timer callbacks ----------------| 

            void timerPublishDistToWaypoint();
            mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped> pub_dist_to_waypoint_;
            std::shared_ptr<TimerType> timer_publish_dist_to_waypoint_;
            int _rate_timer_publish_dist_to_waypoint_;

            void timerPublishSetReference();
            mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped> pub_reference_;
            std::shared_ptr<TimerType>    timer_publisher_reference_;
            int _rate_timer_publisher_reference_;

            void timerCheckSubscribers();
            std::shared_ptr<TimerType> timer_check_subscribers_;
            int _rate_timer_check_subscribers_;

            // | -------------------- service server callbacks ----------------|
            
            bool callbackStartWaypointFollowing(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,  const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_start_waypoints_following_;

            bool callbackStopWaypointFollowing(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,  const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_stop_waypoints_following_;

            bool callbackFlyToFirstWaypoint(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,  const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_fly_to_first_waypoint_;

            // | --------------------- service clients ----------------------|
            mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> srv_client_land_;
            bool               _land_end_;


            // | -------------------- loading waypoints ---------------------- |

            std::vector<mrs_msgs::msg::Reference> waypoints_;
            std::string                      _waypoints_frame_;
            bool                             waypoints_loaded_ = false;
            mrs_msgs::msg::Reference              current_waypoint_;
            std::mutex                       mutex_current_waypoint_;
            int                              idx_current_waypoint_;
            int                              n_waypoints_;
            int                              _n_loops_;
            int                              c_loop_;
            std::mutex                       mutex_waypoint_idle_time_;
            Eigen::MatrixXd                _offset_;

            // |----------dynamic reconfigure ----------------------------|
 
            std::mutex      mutex_dynamic_reconfigure_;        
            template <typename T>
            void callbackDynamicReconfigure(const std::string& param_name, const T& value);


            // | --------------------- waypoint idling -------------------- |

            bool       is_idling_ = false;
            std::shared_ptr<TimerType> timer_idling_;
            rclcpp::TimerBase::SharedPtr timer_intialization_;
            int     _waypoint_idle_time_;
            double     _waypoint_desired_dist_;
            void timerIdling();

            // |------------ support functions -------------|
            std::vector<mrs_msgs::msg::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);
            
            void offsetPoints(std::vector<mrs_msgs::msg::Reference>& points, const Eigen::MatrixXd& offset);
            
            double distance(const mrs_msgs::msg::Reference& waypoint, const geometry_msgs::msg::Pose& pose);
            
    };

    ExampleWaypointFlier::ExampleWaypointFlier(rclcpp::NodeOptions options) : Node("example_waypoint_flier", options){        
        timer_intialization_ = create_wall_timer(std::chrono::duration<double>(1.0),std::bind(&ExampleWaypointFlier::intialize,this));
    }

    void ExampleWaypointFlier::intialize(){
        node_ = this->shared_from_this();
        clock_ = node_->get_clock();

        mrs_lib::ParamLoader param_loader(node_);

        dynparam_mgr_ = std::make_shared<mrs_lib::DynparamMgr>(node_, mutex_dynamic_reconfigure_);

        param_loader.addYamlFileFromParam("config");

        dynparam_mgr_->get_param_provider().copyYamls(param_loader.getParamProvider());

        const mrs_lib::DynparamMgr::update_cbk_t<int> cbk = std::bind(&ExampleWaypointFlier::callbackDynamicReconfigure<int>, this, "waypoint_idle_time", std::placeholders::_1);
        
        const auto result = dynparam_mgr_->register_param("waypoint_idle_time", &_waypoint_idle_time_, cbk);
        std::cout << "Dynamic param loaded : " <<  result << std::endl;

        param_loader.loadParam("uav_name",_uav_name_);
        param_loader.loadParam("n_loops", _n_loops_);
        param_loader.loadParam("waypoint_desired_distance", _waypoint_desired_dist_);
        // param_loader.loadParam("waypoint_idle_time", _waypoint_idle_time_);
        param_loader.loadParam("waypoints_frame", _waypoints_frame_);
        param_loader.loadParam("rate/publish_dist_to_waypoint", _rate_timer_publish_dist_to_waypoint_);
        param_loader.loadParam("rate/publish_reference", _rate_timer_publisher_reference_);
        param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
        
        /* load waypoints as a half-dynamic matrix from config file */
        Eigen::MatrixXd waypoint_matrix;
        param_loader.loadMatrixDynamic("waypoints", waypoint_matrix, -1, 4);  // -1 indicates the dynamic dimension
        waypoints_            = matrixToPoints(waypoint_matrix);
        n_waypoints_          = waypoints_.size();
        waypoints_loaded_     = true;
        idx_current_waypoint_ = 0;
        c_loop_               = 0;
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),"[ExampleWaypointFlier]: " << n_waypoints_ << " waypoints loaded");
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(),"[ExampleWaypointFlier]: " << _n_loops_ << " loops requested");

        /* load offsets of all waypoints as statics matrix from the config file and adjust waypoints accordingly.*/
        param_loader.loadMatrixKnown("offset", _offset_, 1,4);

        if (!param_loader.loadedSuccessfully()){
            RCLCPP_ERROR(node_->get_logger(),"[ExampleWaypointFlier]: failed to load non-optional parameters!");
            rclcpp::shutdown();
        }

        // | ----- initialise subscribers ------|
        mrs_lib::SubscriberHandlerOptions shopts; 
        shopts.node = node_;
        shopts.node_name = "ExampleWaypointFlier";
        shopts.no_message_timeout = rclcpp::Duration(1,0);
        shopts.threadsafe = true;
        shopts.autostart          = true;

        sh_odometry_             = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odom_in");
        sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in",
                                                                                                    &ExampleWaypointFlier::callbackControlManagerDiag, this);
        // | ------------------ initialize publishers ----------------- |

        pub_dist_to_waypoint_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_,"~/dist_to_waypoint_out");
        pub_reference_        = mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped>(node_,"~/reference_out");

        
        // | ----------------- intialize timers ---------------------|
        mrs_lib::TimerHandlerOptions opts_autostart;
        opts_autostart.node = node_;
        opts_autostart.autostart = true;
        {   
            std::function<void()> callback_fn = std::bind(&ExampleWaypointFlier::timerPublishDistToWaypoint, this);
            timer_publish_dist_to_waypoint_  = std::make_shared<TimerType> (opts_autostart, rclcpp::Rate(_rate_timer_publish_dist_to_waypoint_,clock_),callback_fn);
        }

        {
            std::function<void()> callback_fn = std::bind(&ExampleWaypointFlier::timerCheckSubscribers, this);
            timer_check_subscribers_ = std::make_shared<TimerType> (opts_autostart,rclcpp::Rate(_rate_timer_check_subscribers_,clock_),callback_fn);
        }

        mrs_lib::TimerHandlerOptions opts_no_autostart;
        opts_no_autostart.node = node_;
        opts_no_autostart.autostart = false;

        {
            std::function<void()> callback_fn = std::bind(&ExampleWaypointFlier::timerPublishSetReference, this);
            timer_publisher_reference_ = std::make_shared<TimerType> (opts_no_autostart, rclcpp::Rate(_rate_timer_publisher_reference_,clock_),callback_fn);
        }

        // | ----------------- initialize service servers -------------|
        srv_server_start_waypoints_following_ = node_->create_service<std_srvs::srv::Trigger>("start_waypoints_following_in",std::bind(&ExampleWaypointFlier::callbackStartWaypointFollowing, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS());

        srv_server_stop_waypoints_following_ = node_->create_service<std_srvs::srv::Trigger>("stop_waypoints_following_in",std::bind(&ExampleWaypointFlier::callbackStopWaypointFollowing, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS());

        srv_server_fly_to_first_waypoint_ = node_->create_service<std_srvs::srv::Trigger>("fly_to_first_waypoint_in",std::bind(&ExampleWaypointFlier::callbackFlyToFirstWaypoint, this, std::placeholders::_1, std::placeholders::_2), rclcpp::SystemDefaultsQoS());
        
        
        // | ------------------ initialize service clients ------------|
        srv_client_land_ = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_,"land_out");        

        RCLCPP_INFO_ONCE(node_->get_logger(),"[ExampleWaypointFlier]: initialized");
        is_initialized_ = true;
        timer_intialization_->cancel();

    }

    /* |------------------ msg callback opts_autostart --------------------------| */

    /* callbackControlManagerDiag() //{ */

    void ExampleWaypointFlier::callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr diagnostics) {

        /* do not continue if the component is not initialized */
        if (!is_initialized_) {
            return;
        }

        RCLCPP_INFO_ONCE(node_->get_logger(),"[ExampleWaypointFlier]: Received first control manager diagnostics msg");

        // get the variable under the mutex
        mrs_msgs::msg::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

        // extract the pose part of the odometry
        geometry_msgs::msg::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

        double dist = distance(current_waypoint, current_pose);
        // RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Distance to waypoint: %.2f", dist);

        if (have_goal_ && !diagnostics->tracker_status.have_goal) {
            have_goal_ = false;

            if (dist < _waypoint_desired_dist_) {
                waypoint_reached_ = true;
                RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Waypoint reached.");

                /* start idling at the reached waypoint */

                {
                    mrs_lib::TimerHandlerOptions timer_opts_start;
                    timer_opts_start.node      = node_;
                    timer_opts_start.autostart = true;
                    // makes the timer run only once
                    timer_opts_start.oneshot = true;
                    is_idling_ = true;
                    std::function<void()> callback_fcn = std::bind(&ExampleWaypointFlier::timerIdling, this);
                    // auto& param_provider_ = dynam_mgr.get_param_provider();
                    // const auto result = param_provider_.getParam("waypoint_idle_time", _waypoint_idle_time_);

                    timer_idling_ = std::make_shared<TimerType> (timer_opts_start, rclcpp::Rate(rclcpp::Duration(1, 0), clock_), callback_fcn);
                }
                

                RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Idling for %d seconds.", _waypoint_idle_time_);
            }
        }
        
    } 

    // | -------------------- timer callback ----------------------|

    /* timerPublishSetReference() //{*/
    void ExampleWaypointFlier::timerPublishSetReference() {

        if (!is_initialized_) {
            return;
        }

        /* return if the uav is still flying to the previous waypoints */
        if (have_goal_) {
            return;
        }

        /* return if the UAV is idling at a waypoint */
        if (is_idling_) {
            return;
        }

        /* shutdown node after flying through all the waypoints (call land service before) */
        if (idx_current_waypoint_ >= n_waypoints_) {

            c_loop_++;

            RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Finished loop %d/%d", c_loop_, _n_loops_);

            if (c_loop_ >= _n_loops_) {

            RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Finished %d loops of %d waypoints.", _n_loops_, n_waypoints_);

            if (_land_end_) {
                RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Calling land service.");
                std::shared_ptr<std_srvs::srv::Trigger::Request> req = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto res = srv_client_land_.callSync(req);
            }

            RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Shutting down.");
            rclcpp::shutdown();
            return;

            } else {
                RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Starting loop %d/%d", c_loop_ + 1, _n_loops_);
                idx_current_waypoint_ = 0;
            }
        }

        /* create new waypoint msg */
        mrs_msgs::msg::ReferenceStamped new_waypoint;

        // set the frame id in which the reference is expressed
        new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
        new_waypoint.header.stamp    = clock_->now();

        new_waypoint.reference = waypoints_.at(idx_current_waypoint_);

        // set the variable under the mutex
        mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(idx_current_waypoint_), current_waypoint_);

        RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Flying to waypoint %d: x: %.2f y: %.2f z: %.2f heading: %.2f", idx_current_waypoint_ + 1, new_waypoint.reference.position.x,
                new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.heading);

        try {
            pub_reference_.publish(new_waypoint);
        }
        catch (...) {
            RCLCPP_ERROR(node_->get_logger(),"Exception caught during publishing set reference");
        }

        if (waypoint_reached_) {
            idx_current_waypoint_++;
            waypoint_reached_ = false;
        }

        have_goal_ = true;
    }
    //}

    /* timerPublishDistToWayPoint() //{*/
    void ExampleWaypointFlier::timerPublishDistToWaypoint() {

        if (!is_initialized_) {
            return;
        }

        /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
        if (!have_goal_) {
            return;
        }

        // this routine can not work without the odometry
        if (!sh_odometry_.hasMsg()) {
            return;
        }

        // get the variable under the mutex
        mrs_msgs::msg::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

        // extract the pose part of the odometry
        geometry_msgs::msg::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

        double dist = distance(current_waypoint, current_pose);
        RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Distance to waypoint is given as ::: %.2f", dist);

        mrs_msgs::msg::Float64Stamped dist_msg;
        // it is important to set the frame id correctly !!
        dist_msg.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
        dist_msg.header.stamp    = clock_->now();
        dist_msg.value           = dist;

        try {
            pub_dist_to_waypoint_.publish(dist_msg);
        }
        catch (...) {
            RCLCPP_ERROR(node_->get_logger(),"Exception caught during publishing dist to waypoint");
        }
    }

    // }

    /* timerCheckSubscribers() //{ */

    void ExampleWaypointFlier::timerCheckSubscribers() {

        if (!is_initialized_) {
            return;
        }

        if (!sh_odometry_.hasMsg()) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_ , 1.0, "[ExampleWaypointFlier]: Not received uav odom msg since node launch.");
        }

        if (!sh_control_manager_diag_.hasMsg()) {   
            RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_, 1.0, "[ExampleWaypointFlier]: Not received tracker diagnostics msg since node launch.");
        }
    }

    void ExampleWaypointFlier::timerIdling(){     

      auto waypoint_idle_time = mrs_lib::get_mutexed(mutex_dynamic_reconfigure_, _waypoint_idle_time_);

      std::chrono::seconds duration_seconds(waypoint_idle_time);
      rclcpp::sleep_for(std::chrono::duration(duration_seconds));

      RCLCPP_INFO(node_->get_logger(),"[ExampleWaypointFlier]: Idling finished");
      is_idling_ = false;       
        
    }  

    // | ------------------- service callbacks --------------------|
    /* //{ callbackStartWaypointFollowing() */

    bool ExampleWaypointFlier::callbackStartWaypointFollowing([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        if (!is_initialized_) {

            response->success = false;
            response->message = "Waypoint flier not initialized!";
            RCLCPP_WARN(node_->get_logger(),"[ExampleWaypointFlier]: Cannot start waypoint following, nodelet is not initialized.");
            return true;
        }

        if (waypoints_loaded_) {

            timer_publisher_reference_->start();

            RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Starting waypoint following.");

            response->success = true;
            response->message = "Starting waypoint following.";

        } else {

            RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot start waypoint following, waypoints are not set.");
            response->success = false;
            response->message = "Waypoints not set.";
        }

        return true;
    }

    //}

    /* //{ callbackStopWaypointFollowing() */

    bool ExampleWaypointFlier::callbackStopWaypointFollowing([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        if (!is_initialized_) {

            response->success = false;
            response->message = "Waypoint flier not initialized!";
            RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot stop waypoint following, nodelet is not initialized.");
            return true;
        }

        timer_publisher_reference_->stop();

        RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Waypoint following stopped.");

        response->success = true;
        response->message = "Waypoint following stopped.";

        return true;
    }

    //}

    /* //{ callbackFlyToFirstWaypoint() */

    bool ExampleWaypointFlier::callbackFlyToFirstWaypoint([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

        if (!is_initialized_) {

            response->success = false;
            response->message = "Waypoint flier not initialized!";
            RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot start waypoint following, nodelet is not initialized.");

            return true;
        }

        if (waypoints_loaded_) {

            /* create new waypoint msg */
            mrs_msgs::msg::ReferenceStamped new_waypoint;

            // it is important to set the frame id correctly !!
            new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
            new_waypoint.header.stamp    = clock_->now();
            new_waypoint.reference       = waypoints_.at(0);

            mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(0), current_waypoint_);

            // set the variable under the mutex

            idx_current_waypoint_ = 0;
            c_loop_               = 0;

            have_goal_ = true;

            try {
                pub_reference_.publish(new_waypoint);
            }
            catch (...) {
                RCLCPP_ERROR(node_->get_logger(), "Exception caught during publishing new waypoint");
            }

            std::stringstream ss;
            ss << "Flying to first waypoint: x: " << new_waypoint.reference.position.x << ", y: " << new_waypoint.reference.position.y
            << ", z: " << new_waypoint.reference.position.z << ", heading: " << new_waypoint.reference.heading;

            RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1.0, "[ExampleWaypointFlier]: " << ss.str());

            response->success = true;
            response->message = ss.str();

        } else {

            RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot fly to first waypoint, waypoints not loaded!");

            response->success = false;
            response->message = "Waypoints not loaded";
        }

        return true;
    }


    // | ------------------- dynamic callbacks --------------------|
    template <typename T>
    void ExampleWaypointFlier::callbackDynamicReconfigure([[maybe_unused]] const std::string& param_name, const T& value){

        if (!is_initialized_)
            return;

        RCLCPP_INFO(node_->get_logger(),
            "[ExampleWaypointFlier]:"
            "Reconfigure Request: "
            "Waypoint idle time: %d",
            value);

        {
            std::scoped_lock lock(mutex_waypoint_idle_time_);
            _waypoint_idle_time_ = value;
        }
        
    }

    // | -------------------- support functions ------------------- |

    /* matrixToPoints() //{ */

    std::vector<mrs_msgs::msg::Reference> ExampleWaypointFlier::matrixToPoints(const Eigen::MatrixXd& matrix) {

        std::vector<mrs_msgs::msg::Reference> points;

        for (int i = 0; i < matrix.rows(); i++) {

            mrs_msgs::msg::Reference point;
            point.position.x = matrix.row(i)(0);
            point.position.y = matrix.row(i)(1);
            point.position.z = matrix.row(i)(2);
            point.heading    = matrix.row(i)(3);

            points.push_back(point);
        }

        return points;
    }

    // }

    /* offsetPoints // {*/
    void ExampleWaypointFlier::offsetPoints(std::vector<mrs_msgs::msg::Reference>& points, const Eigen::MatrixXd& offset) {

        for (size_t i = 0; i < points.size(); i++) {

            points.at(i).position.x += offset(0);
            points.at(i).position.y += offset(1);
            points.at(i).position.z += offset(2);
            points.at(i).heading += offset(3);
        }
    }
    // }

    /* distance() //{*/

    double ExampleWaypointFlier::distance(const mrs_msgs::msg::Reference& waypoint, const geometry_msgs::msg::Pose& pose){

        return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
    }

    
} // namespace waypoint flier

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(example_waypoint_flier::ExampleWaypointFlier)




