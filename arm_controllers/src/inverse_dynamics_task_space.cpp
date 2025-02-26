// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>          // get kdl tree from urdf
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
// #include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define num_taskspace 6

namespace arm_controllers
{
class InverseDynamics_Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
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

        // 1.2 Gain
        // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/inverse_dynamics_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/inverse_dynamics_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/inverse_dynamics_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/inverse_dynamics_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

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

        // 4.5 forward kinematics solver 초기화
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);
        qddot_.data =  Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

        desired_.data = Eigen::VectorXd::Zero(n_joints_);

        x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);
        x_cmd_.data(0) = 0.0;
        x_cmd_.data(1) = -0.32;
        x_cmd_.data(2) = 0.56;

        for (size_t i = 0; i < num_taskspace; i++)
        {
           ex_(i) = 0;
        }

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        J_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_xd_ = n.advertise<std_msgs::Float64MultiArray>("xd", 1000);
        pub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000);
        pub_ex_ = n.advertise<std_msgs::Float64MultiArray>("ex", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber
        sub = n.subscribe("command", 1000, &InverseDynamics_Controller::commandCB, this);
        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
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

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Inverse Dynamics State Space Controller");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
            qddot_(i) = joints_[i].getEffort();
        }

        // 0.3 end-effector state by Compute forward kinematics (x_,xdot_)
        fk_pos_solver_->JntToCart(q_, x_);
        xdot_ = J_.data * qdot_.data;

        // ********* 1. Desired Trajectory Generation in task space *********

        // *** 1.1 Desired Trajectory in taskspace ***
        xd_.p(0) = x_cmd_(0);
        xd_.p(1) = x_cmd_(1);
        xd_.p(2) = x_cmd_(2);
        xd_.M = KDL::Rotation(KDL::Rotation::RPY(x_cmd_(3), x_cmd_(4), x_cmd_(5)));

        // ********* 2. Inverse Kinematics *********
        // *** 2.0 Error Definition in Task Space ***
        ex_temp_ = diff(x_, xd_);

        ex_(0) = ex_temp_(0);
        ex_(1) = ex_temp_(1);
        ex_(2) = ex_temp_(2);
        ex_(3) = ex_temp_(3);
        ex_(4) = ex_temp_(4);
        ex_(5) = ex_temp_(5);

        ex_dot_ = xd_dot_ - xdot_;

        // *** 2.1 computing Jacobian J(q) ***
        jnt_to_jac_solver_->JntToJac(q_, J_);

        // *** 2.2 computing Jacobian transpose/inversion ***
        J_transpose_ = J_.data.transpose();
        J_inv_ = J_.data.inverse();

        // *** 2.2 computing Jacobian derivative***
        Jdot_ = J_.data.derived();

        // ********* 3. Motion Controller in Joint Space*********
        // *** 3.1 Error Definition in Joint Space ***
        e_.data = qd_.data - q_.data;
        e_dot_.data = qd_dot_.data - qdot_.data;
        e_ddot_.data = qd_ddot_.data - qddot_.data;
        e_int_.data = qd_.data - q_.data; // (To do: e_int 업데이트 필요요)

        // *** 3.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_);

        // *** 3.3 Apply Torque Command to Actuator ***
        // Feedback
        sum_e = xd_ddot_ + Kd_.data.cwiseProduct(ex_dot_) + Kp_.data.cwiseProduct(ex_) - Jdot_*qdot_.data;

        // velocity control
        aux_d_.data = M_.data * (J_inv_*sum_e) ;
        comp_d_.data = C_.data.cwiseProduct(qdot_.data) + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;

        // ISHIRA: Manipulation
        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
        }

        // ********* 4. data 저장 *********
        save_data();

        // ********* 5. state 출력 *********
        print_state();
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

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

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(3);
        SaveData_[40] = e_dot_(4);
        SaveData_[41] = e_dot_(5);
        SaveData_[42] = e_dot_(6);

        // Error intergal value in joint space (unit: rad*sec)
        SaveData_[43] = e_int_(0);
        SaveData_[44] = e_int_(1);
        SaveData_[45] = e_int_(2);
        SaveData_[46] = e_int_(3);
        SaveData_[47] = e_int_(4);
        SaveData_[48] = e_int_(5);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);

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

            printf("*** Command from Subscriber in Task Space (unit: m) ***\n");
            printf("x_cmd: %f, ", x_cmd_(0));
            printf("y_cmd: %f, ", x_cmd_(1));
            printf("z_cmd: %f, ", x_cmd_(2));
            printf("r_cmd: %f, ", x_cmd_(3));
            printf("p_cmd: %f, ", x_cmd_(4));
            printf("y_cmd: %f\n", x_cmd_(5));
            printf("\n");

            printf("*** Desired Position in Joint Space (unit: deg) ***\n");
            printf("qd(1): %f, ", qd_(0) * R2D);
            printf("qd(2): %f, ", qd_(1) * R2D);
            printf("qd(3): %f, ", qd_(2) * R2D);
            printf("qd(4): %f, ", qd_(3) * R2D);
            printf("qd(5): %f, ", qd_(4) * R2D);
            printf("qd(6): %f\n", qd_(5) * R2D);
            printf("\n");

            printf("*** Actual Position in Joint Space (unit: deg) ***\n");
            printf("q(1): %f, ", q_(0) * R2D);
            printf("q(2): %f, ", q_(1) * R2D);
            printf("q(3): %f, ", q_(2) * R2D);
            printf("q(4): %f, ", q_(3) * R2D);
            printf("q(5): %f, ", q_(4) * R2D);
            printf("q(6): %f\n", q_(5) * R2D);
            printf("\n");

            printf("*** Desired Position in Task Space (unit: m) ***\n");
            printf("xd: %f, ", xd_.p(0));
            printf("yd: %f, ", xd_.p(1));
            printf("zd: %f\n", xd_.p(2));
            printf("\n");

            printf("*** Actual Position in Task Space (unit: m) ***\n");
            printf("x: %f, ", x_.p(0));
            printf("y: %f, ", x_.p(1));
            printf("z: %f\n", x_.p(2));
            printf("\n");

            // printf("*** Desired Orientation in Task Space (unit: ??) ***\n");
            // printf("xd_(0): %f, ", xd_.M(KDL::Rotation::RPY(0));
            // printf("xd_(1): %f, ", xd_.M(KDL::Rotation::RPY(1));
            // printf("xd_(2): %f\n", xd_.M(KDL::Rotation::RPY(2));
            // printf("\n");

            // printf("*** Actual Orientation in Task Space (unit: ??) ***\n");
            // printf("x_(0): %f, ", x_.M(KDL::Rotation::RPY(0));
            // printf("x_(1): %f, ", x_.M(KDL::Rotation::RPY(1));
            // printf("x_(2): %f\n", x_.M(KDL::Rotation::RPY(2));
            // printf("\n");

            printf("*** Desired Translation Velocity in Task Space (unit: m/s) ***\n");
            printf("xd_dot: %f, ", xd_dot_(0));
            printf("yd_dot: %f, ", xd_dot_(1));
            printf("zd_dot: %f\n", xd_dot_(2));
            printf("\n");

            printf("*** Actual Translation Velocity in Task Space (unit: m/s) ***\n");
            printf("xdot: %f, ", xdot_(0));
            printf("ydot: %f  ", xdot_(1));
            printf("zdot: %f\n", xdot_(2));
            printf("\n");

            printf("*** Desired Angular Velocity in Task Space (unit: rad/s) ***\n");
            printf("rd_dot: %f, ", xd_dot_(3));
            printf("pd_dot: %f, ", xd_dot_(4));
            printf("yd_dot: %f\n", xd_dot_(5));
            printf("\n");

            printf("*** Actual Angular Velocity in Task Space (unit: rad/s) ***\n");
            printf("r_dot: %f, ", xdot_(3));
            printf("p_dot: %f  ", xdot_(4));
            printf("y_dot: %f\n", xdot_(5));
            printf("\n");

            printf("*** Desired Rotation Matrix of end-effector ***\n");
            printf("%f, ",xd_.M(0,0));
            printf("%f, ",xd_.M(0,1));
            printf("%f\n",xd_.M(0,2));
            printf("%f, ",xd_.M(1,0));
            printf("%f, ",xd_.M(1,1));
            printf("%f\n",xd_.M(1,2));
            printf("%f, ",xd_.M(2,0));
            printf("%f, ",xd_.M(2,1));
            printf("%f\n",xd_.M(2,2));
            printf("\n");

            printf("*** Actual Rotation Matrix of end-effector ***\n");
            printf("%f, ",x_.M(0,0));
            printf("%f, ",x_.M(0,1));
            printf("%f\n",x_.M(0,2));
            printf("%f, ",x_.M(1,0));
            printf("%f, ",x_.M(1,1));
            printf("%f\n",x_.M(1,2));
            printf("%f, ",x_.M(2,0));
            printf("%f, ",x_.M(2,1));
            printf("%f\n",x_.M(2,2));
            printf("\n");

            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("q1: %f, ", R2D * e_(0));
            printf("q2: %f, ", R2D * e_(1));
            printf("q3: %f, ", R2D * e_(2));
            printf("q4: %f, ", R2D * e_(3));
            printf("q5: %f, ", R2D * e_(4));
            printf("q6: %f\n", R2D * e_(5));
            printf("\n");

            printf("*** Task Space Position Error (unit: mm) ***\n");
            printf("x: %f, ", ex_(0)*1000);
            printf("y: %f, ", ex_(1)*1000);
            printf("z: %f\n", ex_(2)*1000);
            printf("\n");

            printf("*** Task Space Orientation Error ?? (unit: deg) ***\n");
            printf("r: %f, ", ex_(3)*R2D);
            printf("p: %f, ", ex_(4)*R2D);
            printf("y: %f\n", ex_(5)*R2D);
            printf("\n");

            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;

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
    Eigen::MatrixXd Jdot_;
    Eigen::MatrixXd J_inv_;
    Eigen::Matrix<double, num_taskspace, num_taskspace> J_transpose_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    // boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_; //Solver to compute the forward kinematics (velocity)
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_, qddot_;
    KDL::JntArray e_, e_dot_, e_ddot_, e_int_;
    KDL::JntArray desired_;


    // Task Space State
    // ver. 01
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist ex_temp_;

    // KDL::Twist xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_;
    Eigen::Matrix<double, num_taskspace, 1> xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> xdot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_dot_, ex_int_, sum_e, blah;

    // Input
    KDL::JntArray x_cmd_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_xd_, pub_x_, pub_ex_;
    ros::Publisher pub_SaveData_;

    // ros subscriber
    ros::Subscriber sub;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::InverseDynamics_Controller, controller_interface::ControllerBase)