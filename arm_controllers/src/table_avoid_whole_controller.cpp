// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/PointCloud2.h>

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

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <utils/pseudo_inversion.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define num_taskspace 6
#define Q_Star 0.05
#define Rep_Gradient 320

namespace arm_controllers
{
    class TableAvoidWhole_Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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
            state_names_.push_back("x");
            state_names_.push_back("y");
            state_names_.push_back("z");
            state_names_.push_back("roll");
            state_names_.push_back("pitch");
            state_names_.push_back("yaw");

            if (!n.getParam("links", link_names_))
            {
                ROS_ERROR("Could not find joint name");
                return false;
            }

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
                ROS_INFO_STREAM(root_name);
            }

            // PIDS////////////////////////////////////////////////////////////////

            // pids
            pids_.resize(state_names_.size());
            for (size_t i=0; i<state_names_.size(); i++)
            {
                // Load PID Controller using gains set on parameter server
                if (!pids_[i].init(ros::NodeHandle(n, "gains/" + state_names_[i] + "/pid")))
                {
                    ROS_ERROR_STREAM("Failed to load PID parameters from " << state_names_[i] + "/pid");
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
            n_joints_ = kdl_chain_.getNrOfJoints();
            // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
            tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

            q_att_.data = Eigen::VectorXd::Zero(n_joints_);
            f_rep_.data = Eigen::VectorXd::Zero(num_taskspace);
            q_rep_.data = Eigen::VectorXd::Zero(n_joints_);
            d_q_.data = Eigen::VectorXd::Zero(n_joints_);

            // xd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            // qd_.data = Eigen::VectorXd::Zero(n_joints_);
            // qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            // qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
            // qd_old_.data = Eigen::VectorXd::Zero(n_joints_);
            // q_init_.data = Eigen::VectorXd::Zero(n_joints_);

            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);
            qC_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            eC_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            // ad_dot_.data = Eigen::VectorXd::Zero(n_joints_);

            // e_.data = Eigen::VectorXd::Zero(n_joints_);
            // e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
            // e_int_.data = Eigen::VectorXd::Zero(n_joints_);

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
            poly_pub = n.advertise<visualization_msgs::Marker>("obstacle", 0);

            // pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

            x_cmd_.data = Eigen::VectorXd::Zero(num_taskspace);
            x_cmd_(0) = 0.1;
            x_cmd_(1) = -0.32;
            x_cmd_(2) = 0.7;
            x_cmd_(3) = 0;

            x_obs_.data = Eigen::VectorXd::Zero(num_taskspace);
            x_obs_(0) = 10;
            x_obs_(1) = 10;
            x_obs_(2) = 10;

            particles_x[0] = 0.0;
            particles_y[0] = -0.6;
            particles_z[0] = 0.2;

            double step_x = 0.4/49;
            double step_y = 0.4/49;
            double step_z = 0.05/49;
            for (size_t i = 0; i<49; i++){
                particles_x[i+1] = particles_x[i] + step_x;
                particles_y[i+1] = particles_y[i] + step_y;
                particles_z[i+1] = particles_z[i] + step_z;
            }

            // 6.2 subsriber
            sub = n.subscribe("command", 1000, &TableAvoidWhole_Controller::commandCB, this);
            obs_sub = n.subscribe("obstacePosition", 1000, &TableAvoidWhole_Controller::obstacleCB, this);
            table_sub = n.subscribe("particles_pcl", 1000, &TableAvoidWhole_Controller::particlesCB, this);
            return true;
        }

        void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
        {
            if (msg->data.size() != state_names_.size())
            {
                ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
                return;
            }
            else
            {
                for (size_t i = 0; i < state_names_.size(); i++)
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

        void particlesCB(const sensor_msgs::PointCloud2 &msg)
        {
            particles_ = msg;
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

            for (int i = 0; i < n_joints_; i++)
            {
                Kp_(i) = pids_[i].getGains().p_gain_;
                Kd_(i) = pids_[i].getGains().d_gain_;
                Ki_(i) = pids_[i].getGains().i_gain_;
            }

            // ********* 1. Desired Trajecoty in Joint Space *********
            fk_pos_solver_->JntToCart(q_,x_);
            // *** 1.1 Desired Trajectory in taskspace ***
            xd_.p(0) = x_cmd_(0);
            xd_.p(1) = x_cmd_(1);
            xd_.p(2) = x_cmd_(2);
            xd_.M = KDL::Rotation(KDL::Rotation::RPY(x_cmd_(3), x_cmd_(4), x_cmd_(5)));

            ex_temp_ = diff(x_, xd_);

            ex_(0) = ex_temp_(0);
            ex_(1) = ex_temp_(1);
            ex_(2) = ex_temp_(2);
            ex_(3) = ex_temp_(3);
            ex_(4) = ex_temp_(4);
            ex_(5) = ex_temp_(5);

            jnt_to_jac_solver_->JntToJac(q_, J_);

            // ********* 2. Calculating Potentials *********
            // *** 2.0 Attractive Potential **
            J_inv_ = J_.data.inverse();

            aux_2_d_.data = Kp_.data.cwiseProduct(ex_);

            q_att_.data = J_inv_ * aux_2_d_.data;

            qC_dot_.data = q_att_.data ;

            // *** 2.1 Repulsive Potential ***
            // FK for segments
            for (int n=1; n<n_joints_+1; n++) {
                Eigen::MatrixXd J_inv_temp;
                std::string root_name, tip_name;
                root_name = "world";
                tip_name = link_names_[n];
                // KDL chain from world to nth link
                if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_temp_))
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

                }
                int n_joints_temp_ = kdl_chain_temp_.getNrOfJoints();
                // Current Joint coordinates upto nth joint
                q_temp_.data = Eigen::VectorXd::Zero(n_joints_temp_);
                for (int i = 0; i < n_joints_temp_; i++) {
                    q_temp_.data[i] = q_.data[i];
                }
                // FK upto nth joint
                fk_pos_solver_temp_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_temp_));
                fk_pos_solver_temp_->JntToCart(q_temp_,x_);
                // Jacobian upto nth joint
                jnt_to_jac_solver_temp_.reset(new KDL::ChainJntToJacSolver(kdl_chain_temp_));
                J_temp_.resize(n_joints_temp_);
                jnt_to_jac_solver_temp_->JntToJac(q_temp_, J_temp_);
                printf("%i", n);
                // Pseudoinverse upto nth joint
                pseudo_inverse(J_temp_.data, J_inv_temp);
                dqc_ = 100;
                for (int i=0; i<50; i++){
                   for (int j=0; j<50; j++) {
                       for (int k=0; k < 50; k++) {
                           double d_;
                           d_ = sqrt(pow(particles_x[i] - x_.p(0), 2) + pow(particles_y[j] - x_.p(1), 2) +
                                     pow(particles_z[k] - x_.p(2), 2));
                           if (d_ < dqc_) {
                               dqc_ = d_;
                               eobs_(0) = x_.p(0) - particles_x[i];
                               eobs_(1) = x_.p(1) - particles_y[i];
                               eobs_(2) = x_.p(2) - particles_z[i];
                               eobs_(3) = 0;
                               eobs_(4) = 0;
                               eobs_(5) = 0;
                           }
                       }
                   }
                }
                if(dqc_ <= Q_Star) {
                   eobs_ = eobs_*(1/dqc_);
                   f_rep_.data =  (Rep_Gradient*((1/dqc_)-(1/Q_Star))*(1/(pow(dqc_,2))))* eobs_;
                   q_temp_.data = J_inv_ * f_rep_.data;
                   for (int i=0; i<n_joints_temp_; i++){
                       qC_dot_.data[i] = qC_dot_.data[i] + q_temp_.data[i];
                   }
                }

            }

            // ********* 3. Motion Controller in Joint Space*********
           // *** 3.0 Compute model(M,C,G) ***
           id_solver_->JntToMass(q_, M_);
           id_solver_->JntToCoriolis(q_, qdot_, C_);
           id_solver_->JntToGravity(q_, G_);

           // *** 3.2 Apply Torque Command to Actuator ***
           // ISHIRA: Stabilizing Linear Control
           eC_dot_.data = qC_dot_.data - qdot_.data;
           aux_d_.data = M_.data * (Kd_.data.cwiseProduct(eC_dot_.data)) ;
           comp_d_.data = C_.data.cwiseProduct(qdot_.data) + G_.data;
           tau_d_.data = aux_d_.data + comp_d_.data;

           // *** 3.3 Manipulation ***
           for (int i = 0; i < n_joints_; i++)
           {
               joints_[i].setCommand(tau_d_(i));
               // joints_[i].setCommand(0.0);
           }

           // ********* 4. data 저장 *********
           // save_data();

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
                // msg_qd_.data.push_back(qd_(i));
                msg_q_.data.push_back(q_(i));
                // msg_e_.data.push_back(e_(i));
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
                //pubPolygon();
                printf("*********************************************************\n\n");
                printf("*** Simulation Time (unit: sec)  ***\n");
                printf("t = %f\n", t);
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

        void pubPolygon()
        {

            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            // marker.ns = "my_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = particles_x[0] + table.width/2;
            marker.pose.position.y = particles_y[0] + table.length/2;
            marker.pose.position.z = particles_z[0] + table.height/2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = table.width;
            marker.scale.y = table.length;
            marker.scale.z = table.height;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            //only if using a MESH_RESOURCE marker type:
            // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
            poly_pub.publish( marker );
        }

    private:
        // others
        double t;

        //Joint handles
        unsigned int n_joints_;                               // joint 숫자
        std::vector<std::string> joint_names_;
        std::vector<std::string> link_names_;
        std::vector<std::string> state_names_;
        std::vector<hardware_interface::JointHandle> joints_; // ??
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

        // kdl
        KDL::Tree kdl_tree_;   // tree?
        KDL::Chain kdl_chain_; // chain?
        KDL::Chain kdl_chain_temp_; // chain?

        // kdl M,C,G
        KDL::JntSpaceInertiaMatrix M_; // intertia matrix

        KDL::JntArray C_;              // coriolis
        KDL::JntArray G_;              // gravity torque vector
        KDL::Vector gravity_;

        // kdl solver
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_, fk_pos_solver_temp_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_, jnt_to_jac_solver_temp_; //Solver to compute the jacobian
        boost::scoped_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_; //Solver to compute inverse kinematics

        // Joint Space State
        // KDL::JntArray qd_, qd_dot_, x_cmd_, xd_dot_, x_obs_;
        // KDL::JntArray qd_old_;
        KDL::JntArray q_, qdot_, qC_dot_, eC_dot_, x_cmd_, x_obs_, q_temp_;
        // KDL::JntArray e_, e_dot_, e_int_;

        // Task Space State
        // ver. 01
        // KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
        KDL::Frame xd_, x_, xobs_dist_;
        std::vector<KDL::Frame> x_vec_;
        KDL::Twist ex_temp_;
        KDL::Twist ex_temp_obs_;

        // KDL::Twist xd_dot_, xd_ddot_;
        Eigen::Matrix<double, num_taskspace, 1> ex_, eobs_;
        Eigen::Matrix<double, num_taskspace, 1> xd_dot_;

        // Input
        KDL::JntArray aux_d_;
        KDL::JntArray aux_2_d_;
        KDL::JntArray comp_d_;
        KDL::JntArray tau_d_;

        // Potential
        KDL::JntArray q_att_;
        KDL::JntArray f_rep_, q_rep_;
        KDL::JntArray d_q_;

        // gains
        KDL::JntArray Kp_, Ki_, Kd_;
        std::vector<control_toolbox::Pid> pids_;

        // kdl and Eigen Jacobian
        KDL::Jacobian J_, J_temp_;
        Eigen::MatrixXd J_inv_;

        // save the data
        double SaveData_[SaveDataMax];
        double dqc_;
        double dx_sqr_, dy_sqr_, dz_sqr_;

        // ros publisher
        ros::Publisher pub_qd_, pub_q_, pub_e_;
        ros::Publisher pub_SaveData_;
        ros::Publisher poly_pub;

        // ros subscriber
        ros::Subscriber sub;
        ros::Subscriber obs_sub;
        ros::Subscriber table_sub;

        // ros message
        std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
        std_msgs::Float64MultiArray msg_SaveData_;
        sensor_msgs::PointCloud2 particles_;

        double d_q;
        double particles_x [50];
        double particles_y [50];
        double particles_z [50];

        struct Table { double width, length, height;} table = {0.4,0.4,0.05};
    };
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::TableAvoidWhole_Controller, controller_interface::ControllerBase)
