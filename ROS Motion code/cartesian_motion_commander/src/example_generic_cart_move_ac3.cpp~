// example_generic_cart_move_ac: 
// wsn, Nov, 2016; updated 12/17
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm_cart_move_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;
    
    nsteps = 10;
    arrival_time = 2.0;
    
    tool_pose_home = cart_motion_commander.get_tool_pose_stamped();//get_tool_pose_stamped
    ROS_INFO("tool pose is: ");
    xformUtils.printPose(tool_pose_home);
    //alter the tool pose:
    std::cout<<"enter 1 to confirm motion to toy area: ";
    int ans;
    std::cin>>ans; 
    

    Eigen::Vector3d b_des,n_des,t_des,O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << -1, 0, 0;
    t_des = b_des.cross(n_des);
    
    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;
    
    O_des<<0.3,-0.1,0.0;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation()= O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

//MOVE TO PICK UP THE TOY

    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine,"system_ref_frame");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj(); 

    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }    

    //start multi-traj planning:
    int nsegs = 0;
   vector<double> arrival_times;


//ACTIVATE VACUUM GRIPPER HOLD ON TOY
    ROS_INFO("Confirmation of vacuum gripper activation: ");
    std::cout<<"enter 1 to approve vacuum gripper activation: ";
    std::cin>>ans; 
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj(); 
        ros::Duration(1.0).sleep();
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }  

//MOVE TO PLACE THE TOY
    tool_pose.pose.position.y+=0.2;
    ROS_INFO("Confirmation of motion to place toy:");
    std::cout<<"enter 1 to approve motion to place toy: ";
    std::cin>>ans; 
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj(); 
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }  

//RELEASE VACUUM GRIPPER HOLD ON TOY
    ROS_INFO("Confirmation of vacuum gripper release: ");
    std::cout<<"enter 1 to approve vacuum gripper release upon arrival: ";
    std::cin>>ans; 
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj(); 
        ros::Duration(1.0).sleep();
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }  
  

//MOVE BACK TO PICK UP ANOTHER TOY
    tool_pose.pose.position.y-=0.2;
     ROS_INFO("Confirmation of motion to return to toy area: ");
    std::cout<<"enter 1 to approve motion back to toy area: ";
    std::cin>>ans; 
    xformUtils.printPose(tool_pose);
    rtn_val=cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps,arrival_time,tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS)  { 
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val=cart_motion_commander.execute_planned_traj(); 
        ros::Duration(1.0).sleep();
    }
    else {
        ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
    }  


/*
    //execute multi-traj:
    double wait_time;
    for (int iseg=0;iseg<nsegs;iseg++) {
        ROS_INFO("commanding seg %d",iseg);
        //wait_time = arrival_times[iseg];
        rtn_val = cart_motion_commander.execute_traj_nseg(iseg);
        //ros::Duration(wait_time).sleep();
    }
*/
    
    return 0;
}


//OLD CODE
/*

   // rtn_val=cart_motion_commander.append_multi_traj_cart_segment(nsteps, arrival_time,tool_pose);
    rtn_val=cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time,tool_pose);
    if (rtn_val != arm_motion_action::arm_interfaceResult::SUCCESS) {
         ROS_WARN("unsuccessful plan; rtn_code = %d",rtn_val);
         exit(1);
    }     
    else {
       nsegs++;
    //   arrival_times.push_back(arrival_time);
    }

*/
