// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <urdf/model.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>          // get kdl tree from urdf
#include <kdl/chaindynparam.hpp>              // inverse dynamics

#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
#include <boost/lexical_cast.hpp>
#include <std_msgs/String.h>
#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define num_taskspace 6

namespace arm_controllers
{
class GravityPD_Controller_VisualServo : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
    public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        loop_count_ = 0;
        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);

        // 2. ********* urdf *********
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
        std::string root_name, tip_name;
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

        // 4.4 jacobian solver 초기화
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);
        // x_cmd_.data(0) = 0.0;
        // x_cmd_.data(1) = 0.0;
        // x_cmd_.data(2) = 0.80;

        x_est_.data = Eigen::VectorXd::Zero(num_taskspace);

        for (size_t i = 0; i < num_taskspace; i++)
        {
            ex_(i) = 0;
        }

         // PIDS////////////////////////////////////////////////////////////////

        // pids
        pids_.resize(n_joints_);
        for (size_t i=0; i<n_joints_; i++)
        {
            // Load PID Controller using gains set on parameter server
            if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
            {
                ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
                return false;
            }
        }

        /////////////////////////////////////////////////////////////////////////

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        J_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());


        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?
        // 6.2 subsriber
        sub = n.subscribe("command", 1000, &GravityPD_Controller_VisualServo::commandCB, this);
        
        //Visual servo camera subscriber
        cam_sub = n.subscribe("/elfin_camera/aruco_tracker/pose", 1000, &GravityPD_Controller_VisualServo::camPoseCB, this);

        
        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        // ROS_INFO("comand o chickenfoot");
        if (msg->data.size() != num_taskspace)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }
        else
        {
            for (size_t i = 0; i < num_taskspace; i++)
            {
                x_cmd_(i) = msg->data[i];
            }
        }
    }
    
    
    void camPoseCB(const geometry_msgs::PoseStamped &msg)
    {
            ros::Rate rate(10.0);
            
            
            // ROS_INFO("chickenfoot");
            try{
                tflistener.lookupTransform("/camera_desired", "/elfin_base",
                                        ros::Time(0), stf);
            }
                catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            
            // x_est_(0) = stf.getOrigin().x();
            // x_est_(1) = stf.getOrigin().y();
            // x_est_(2) = stf.getOrigin().z();
            // x_est_(3) = stf.getRotation().x();
            // x_est_(4) = stf.getRotation().y();
            // x_est_(5) = stf.getRotation().z();
            // x_est_(6) = stf.getRotation().w();
            // rate.sleep();
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
         // 0.2 joint state
        // for (int i = 0; i < n_joints_; i++)
        // {
        //     q_(i) = joints_[i].getPosition();
        //     qdot_(i) = joints_[i].getVelocity();
        // }

        
        ROS_INFO("Starting Gravity Compensation and PD Controller");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        // *** 0.3 computing Jacobian J(q) ***
        jnt_to_jac_solver_->JntToJac(q_, J_);

        // *** 0.4 computing Jacobian transpose/inversion ***
        J_transpose_ = J_.data.transpose();

        // 0.5 end-effector state by Compute forward kinematics (x_,xdot_)
        fk_pos_solver_->JntToCart(q_, x_);
        xdot_ = J_.data * qdot_.data;

        // ********* 1. Desired Trajecoty in Task Space *********
        xd_.p(0) = x_est_(0);
        xd_.p(1) = x_est_(1);
        xd_.p(2) = x_est_(2);

        quat = tf::Quaternion( xd_.p(3),  xd_.p(4),  xd_.p(5),  xd_.p(6));
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        xd_.M = KDL::Rotation(KDL::Rotation::RPY(roll, pitch, yaw));

        // ********* 2. Motion Controller in Joint Space*********
        // *** 2.0 Error Definition in Task Space ***
        ex_temp_ = diff(x_, xd_);

        ex_(0) = ex_temp_(0);
        ex_(1) = ex_temp_(1);
        ex_(2) = ex_temp_(2);
        ex_(3) = ex_temp_(3);
        ex_(4) = ex_temp_(4);
        ex_(5) = ex_temp_(5);

        // *** 2.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_);

        // *** 2.3 Apply Torque Command to Actuator ***

        // ISHIRA: Manipulation
        for (int i = 0; i < n_joints_; i++)
        {
            Kp_(i) = pids_[i].getGains().p_gain_;
            Kd_(i) = pids_[i].getGains().d_gain_;
            Ki_(i) = pids_[i].getGains().i_gain_;
        }

        aux_d_.data = J_transpose_*(Kp_.data.cwiseProduct(ex_)-Kd_.data.cwiseProduct(xdot_));
        tau_d_.data = aux_d_.data + G_.data;

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
        }

        if (loop_count_ % 1 == 0)
        {
            // ********* 3. data 저장 *********
            save_data();

            // ********* 4. state 출력 *********
            // print_state();
        }
        loop_count_++;
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_q_.data.push_back(q_(i));

        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_q_.publish(msg_q_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");
            printf("*** Camera POS WRT world ***\n");

            printf("x_est_(0): %f, ", x_est_(0));
            printf("x_est_(1): %f, ", x_est_(1));
            printf("x_est_(2): %f, ", x_est_(2));
            printf("x_est_(3): %f, ", x_est_(3));
            printf("x_est_(4): %f, ", x_est_(4));
            printf("x_est_(5): %f, ", x_est_(5));
            printf("x_est_(6): %f\n", x_est_(6));
            printf("\n");
            printf("*** Desired Position in Task Space (unit: m) ***\n");
            printf("xd: %f, ", xd_.p(0));
            printf("yd: %f, ", xd_.p(1));
            printf("zd: %f\n", xd_.p(2));
            printf("\n");
            printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
            printf("x_cmd: %f, ", x_cmd_(0));
            printf("y_cmd: %f, ", x_cmd_(1));
            printf("z_cmd: %f, ", x_cmd_(2));
            printf("r_cmd: %f, ", x_cmd_(3));
            printf("p_cmd: %f, ", x_cmd_(4));
            printf("y_cmd: %f\n", x_cmd_(5));
            printf("\n");
            printf("*** Actual Position in Task Space (unit: m) ***\n");
            printf("x: %f, ", x_.p(0));
            printf("y: %f, ", x_.p(1));
            printf("z: %f\n", x_.p(2));
            printf("\n");
            count = 0;
        }
        count++;
    }

private:
    // others
    double t;
    int loop_count_;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl and Eigen Jacobian
    KDL::Jacobian J_;
    Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics

    // Input
    KDL::JntArray x_est_; //marker_pose;

    // Joint Space State
    KDL::JntArray q_, qdot_;

    // Task Space State
    // ver. 01
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist ex_temp_;

    // KDL::Twist xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_;
    Eigen::Matrix<double, num_taskspace, 1> xdot_;

    // Input
    KDL::JntArray x_cmd_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray tau_d_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;
    std::vector<control_toolbox::Pid> pids_;       /**< Internal PID controllers. */

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_q_;
    ros::Publisher pub_SaveData_;

    // ros subsciber
    ros::Subscriber sub;
    ros::Subscriber cam_sub;
    tf::TransformListener tflistener;
    tf::StampedTransform stf;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;


    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Quaternion quat;
    double roll, pitch, yaw;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityPD_Controller_VisualServo, controller_interface::ControllerBase)