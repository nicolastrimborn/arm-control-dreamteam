// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_toolbox/pid.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>          // get kdl tree from urdf
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainiksolverpos_lma.hpp>           // inverse kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define num_taskspace 6
#define Q_star 0.08

namespace arm_controllers
{
class TableAvoid_Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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
        K_att_.resize(n_joints_);

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

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

        ik_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));
        // 4.4 jacobian solver 초기화
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

        // 4.5 forward kinematics solver 초기화
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        f_att_.data = Eigen::VectorXd::Zero(n_joints_);
        f_rep_.data = Eigen::VectorXd::Zero(n_joints_);
        q_rep_.data = Eigen::VectorXd::Zero(n_joints_);
        d_q_ = 0;
        q_star_.data = 360*KDL::deg2rad*Eigen::VectorXd::Ones(n_joints_);
        max_limit_.data = 180*KDL::deg2rad*Eigen::VectorXd::Ones(n_joints_);
        min_limit_.data = -180*KDL::deg2rad*Eigen::VectorXd::Ones(n_joints_);
        K_att_.data = 100.0*Eigen::VectorXd::Ones(n_joints_);
        K_rep_ = 70.0;

        xd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

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

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);
        x_cmd_(0) = 0.0;
        x_cmd_(1) = -0.32;
        x_cmd_(2) = 0.56;

        x_obs_.data = Eigen::VectorXd::Zero(num_taskspace);
        x_obs_(0) = 10;
        x_obs_(1) = 10;
        x_obs_(2) = 10;
        x_obs_(3) = 0;
        x_obs_(4) = 0;
        x_obs_(5) = 0;
        // 6.2 subsriber
        sub = n.subscribe("command", 1000, &TableAvoid_Controller::commandCB, this);
        obs_sub = n.subscribe("obstacePosition", 1000, &TableAvoid_Controller::obstacleCB, this);
        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }
        else
        {
            for (size_t i = 0; i < n_joints_; i++)
            {
                x_cmd_(i) = msg->data[i];
            }
        }
    }

    void obstacleCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }
        else
        {
            for (size_t i = 0; i < n_joints_; i++)
            {
                x_obs_(i) = msg->data[i];
            }
        }
    }


    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Computed Torque Controller");
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
        }
        

        // ********* 1. Desired Trajecoty in Joint Space *********

        // *** 1.1 Desired Trajectory in taskspace ***
        xd_.p(0) = x_cmd_(0);
        xd_.p(1) = x_cmd_(1);
        xd_.p(2) = x_cmd_(2);
        xd_.M = KDL::Rotation(KDL::Rotation::RPY(x_cmd_(3), x_cmd_(4), x_cmd_(5)));

        jnt_to_jac_solver_->JntToJac(q_, J_);
        // *** 1.2 computing inverse kinematics***
        ik_solver_->CartToJnt(q_, xd_, qd_);

        // ********* 2. Motion Controller in Joint Space*********
        // *** 2.0 Potential ***
        // Attractive potential where f_att_ is a velocity
        fk_pos_solver_->JntToCart(q_,x_);

        xd_.p(0) = x_cmd_(0);
        xd_.p(1) = x_cmd_(1);
        xd_.p(2) = x_cmd_(2);
        xd_.M = KDL::Rotation(KDL::Rotation::RPY(x_cmd_(3), x_cmd_(4), x_cmd_(5)));

        xd_dot_(0) = 0;
        xd_dot_(1) = 0;
        xd_dot_(2) = 0;
        xd_dot_(3) = 0;
        xd_dot_(4) = 0;
        xd_dot_(5) = 0;

        ex_temp_ = diff(x_, xd_);

        ex_(0) = ex_temp_(0);
        ex_(1) = ex_temp_(1);
        ex_(2) = ex_temp_(2);
        ex_(3) = ex_temp_(3);
        ex_(4) = ex_temp_(4);
        ex_(5) = ex_temp_(5);

        //std::cout << "error X " << ex_ << std::endl;

        aux_2_d_.data = xd_dot_.data + K_att_.data.cwiseProduct(ex_);

        jnt_to_jac_solver_->JntToJac(q_, J_);

        // *** 2.2 computing Jacobian transpose/inversion ***
        J_inv_ = J_.data.inverse();

        f_att_.data = J_inv_ * aux_2_d_.data;

        // Repulsive potential where f_rep_ is a velocity
        xobs_dist_.p(0) = x_obs_(0);
        xobs_dist_.p(1) = x_obs_(1);
        xobs_dist_.p(2) = x_obs_(2);
        xobs_dist_.M = KDL::Rotation(KDL::Rotation::RPY(x_obs_(3), x_obs_(4), x_obs_(5)));

        ex_temp_obs_ = diff(x_, xobs_dist_);

        eobs_(0) = ex_temp_obs_(0);
        eobs_(1) = ex_temp_obs_(1);
        eobs_(2) = ex_temp_obs_(2);
        eobs_(3) = ex_temp_obs_(3);
        eobs_(4) = ex_temp_obs_(4);
        eobs_(5) = ex_temp_obs_(5);

        d_q_ = sqrt(pow(eobs_(0),2) +pow(eobs_(1),2) + pow(eobs_(2),2));

        eobs_ = eobs_*(1/d_q_);

        if (d_q_<=Q_star){
            f_rep_.data = K_rep_*((1/d_q_)-(1/Q_star))*(pow((1/d_q_), 2))*ex_;
            q_rep_.data = J_inv_ * f_rep_.data;
            qd_dot_.data = f_att_.data + q_rep_.data;
        }
        else{
            qd_dot_.data = f_att_.data;
        }

        // *** 2.1 Error Definition in Joint Space ***
        e_.data = qd_.data - q_.data;
        e_dot_.data = qd_dot_.data - qdot_.data;
        e_int_.data = qd_.data - q_.data; // (To do: e_int 업데이트 필요요)

        // *** 2.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_);

        // *** 2.3 Apply Torque Command to Actuator ***
        for (int i = 0; i < n_joints_; i++)
        {
            Kp_(i) = pids_[i].getGains().p_gain_;
            Kd_(i) = pids_[i].getGains().d_gain_;
            Ki_(i) = pids_[i].getGains().i_gain_;
        }

        // ISHIRA: Stabilizing Linear Control
        aux_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data));
        // ISHIRA: n(q, qdot)
        comp_d_.data = C_.data + G_.data;
        // ISHIRA: Nonlinear Compensation and Decoupling
        tau_d_.data = aux_d_.data + comp_d_.data;


        // ISHIRA: Manipulation
        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
            // joints_[i].setCommand(0.0);
        }

        // ********* 3. data 저장 *********
        save_data();

        // ********* 4. state 출력 *********
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

            printf("*** Desired State in Joint Space (unit: deg) ***\n");
            printf("qd_(0): %f, ", qd_(0)*R2D);
            printf("qd_(1): %f, ", qd_(1)*R2D);
            printf("qd_(2): %f, ", qd_(2)*R2D);
            printf("qd_(3): %f, ", qd_(3)*R2D);
            printf("qd_(4): %f, ", qd_(4)*R2D);
            printf("qd_(5): %f\n", qd_(5)*R2D);
            printf("\n");

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");


            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("%f, ", R2D * e_(0));
            printf("%f, ", R2D * e_(1));
            printf("%f, ", R2D * e_(2));
            printf("%f, ", R2D * e_(3));
            printf("%f, ", R2D * e_(4));
            printf("%f\n", R2D * e_(5));
            printf("\n");

            printf("*** Actual State in State Space (unit: deg) ***\n");
            printf("x: %f, ", x_.p(0));
            printf("y: %f, ", x_.p(1));
            printf("z: %f, ", x_.p(2));
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

    // kdl solver
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
    boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_; //Solver to compute inverse kinematics

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_, x_cmd_, xd_dot_, x_obs_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_;

    // z point
    

    // Task Space State
    // ver. 01
   // KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame xd_, xobs_dist_;
    KDL::Frame x_;
    KDL::Twist ex_temp_;
    KDL::Twist ex_temp_obs_;
    Eigen::Matrix<double, num_taskspace, 1> eobs_;

    // KDL::Twist xd_dot_, xd_ddot_;
    Eigen::Matrix<double, num_taskspace, 1> ex_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;
    KDL::JntArray aux_2_d_;

    // Potential
    KDL::JntArray f_att_;
    KDL::JntArray f_rep_;
    KDL::JntArray q_rep_;
    double d_q_;
    KDL::JntArray q_star_;
    KDL::JntArray min_limit_;
    KDL::JntArray max_limit_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_, K_att_, Kp_E_;
    std::vector<control_toolbox::Pid> pids_;
    double K_rep_;

    // kdl and Eigen Jacobian
    KDL::Jacobian J_;
    Eigen::MatrixXd J_inv_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;

    // ros subscriber
    ros::Subscriber sub;
    ros::Subscriber obs_sub;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::TableAvoid_Controller, controller_interface::ControllerBase)
