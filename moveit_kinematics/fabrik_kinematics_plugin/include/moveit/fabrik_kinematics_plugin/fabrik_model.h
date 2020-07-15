


/* Desc: convert urdf to fabrik model 
    const moveit::core::JointModelGroup* joint_model_group_;
*/

/* Author: Omid Heidari */

#include <fabrik/robot_model/robot_model.h>

#include <moveit/robot_model/robot_model.h>

#include <Eigen/Geometry>


// #include <collada_urdf/collada_urdf.h>

// #include <dae.h>

// install collada_urdf from apt install
// then use rosdep install collada_urdf so that all the dependencies get installed as well

namespace fabrik_kinematics_plugin
{

// convert urdf or the info from joint_model_grouo to a model fabrik needs. Basically the 
// relative displacement between two end points of each link is what fabrik needs 
class FabrikModel
{
public:

FabrikModel()
{
ROS_INFO("-------------------------------- 81");

   fabrik_robot_model = fabrik::makeSimpleRobot3D();
ROS_INFO("-------------------------------- 82");

}

FabrikModel(const moveit::core::JointModelGroup* joint_model_group)
{
    
ROS_INFO("-------------------------------- 51");
    // or make the one for panda manually
    Eigen::Vector3d vec0(0,0,1);
    vec0.normalize();
    Eigen::Affine3d base(Eigen::AngleAxisd(0, vec0));
    base.translation() = Eigen::Vector3d(0, 0, 0.333/2);
ROS_INFO("-------------------------------- 52");

    Eigen::Vector3d vec1(1,0,0);
    vec1.normalize();
    Eigen::Affine3d link1_frame(Eigen::AngleAxisd(-M_PI_2, vec1));
    link1_frame.translation() = Eigen::Vector3d(0, 0, 0.333/2);
    fabrik::Link link1("link1",  link1_frame);
ROS_INFO("-------------------------------- 53");

    Eigen::Vector3d vec2(1,0,0);
    vec2.normalize();
    Eigen::Affine3d link2_frame(Eigen::AngleAxisd(M_PI_2, vec2));
    link2_frame.translation() = Eigen::Vector3d(0, -0.316/2, 0);
    fabrik::Link link2("link2",  link2_frame);
ROS_INFO("-------------------------------- 54");

    Eigen::Vector3d vec3(1,0,0);
    vec3.normalize();
    Eigen::Affine3d link3_frame(Eigen::AngleAxisd(M_PI_2, vec3));
    link3_frame.translation() = Eigen::Vector3d(0.0825, 0, 0.316/2);
    fabrik::Link link3("link3",  link3_frame);
ROS_INFO("-------------------------------- 55");

    Eigen::Vector3d vec4(1,0,0);
    vec4.normalize();
    Eigen::Affine3d link4_frame(Eigen::AngleAxisd(-M_PI_2, vec4));
    link4_frame.translation() = Eigen::Vector3d(-0.0825, 0.384/3, 0);
    fabrik::Link link4("link4",  link4_frame);
ROS_INFO("-------------------------------- 56");

    Eigen::Vector3d vec5(1,0,0);
    vec5.normalize();
    Eigen::Affine3d link5_frame(Eigen::AngleAxisd(M_PI_2, vec5));
    link5_frame.translation() = Eigen::Vector3d(0, 0, 0.384);
    fabrik::Link link5("link5",  link5_frame);
ROS_INFO("-------------------------------- 57");

    Eigen::Vector3d vec6(1,0,0);
    vec6.normalize();
    Eigen::Affine3d link6_frame(Eigen::AngleAxisd(M_PI_2, vec6));
    link6_frame.translation() = Eigen::Vector3d(0.088, -0.5, 0);
    fabrik::Link link6("link6",  link6_frame);
ROS_INFO("-------------------------------- 58");

    Eigen::Vector3d vec7(1,0,0);
    vec7.normalize();
    Eigen::Affine3d link7_frame(Eigen::AngleAxisd(0, vec7));
    link7_frame.translation() = Eigen::Vector3d(0, 0, 1.07);
    fabrik::Link link7("link7",  link7_frame);
ROS_INFO("-------------------------------- 59");

    std::string robot_name = "panda robot";
    std::vector<fabrik::Link> chain;
    chain.push_back(link1);
    chain.push_back(link2);
    chain.push_back(link3);
    chain.push_back(link4);
    chain.push_back(link5);
    chain.push_back(link6);
    chain.push_back(link7);
ROS_INFO("-------------------------------- 60");
    fabrik_robot_model = std::make_shared<fabrik::RobotModel>(robot_name, base, chain);
    ROS_INFO("-------------------------------- 61");
}

FabrikModel(const urdf::Model& my_model)
{
    // boost::shared_ptr<DAE> dom;

    // if (!collada_urdf::colladaFromUrdfModel(my_model, dom)){
    //     ROS_ERROR("Failed to construct COLLADA DOM");
    //     return false;
    // }

} 


fabrik::RobotModelPtr fabrik_robot_model;

};

}