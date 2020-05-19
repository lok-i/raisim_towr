#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#define PI  3.141592
//MONOPED LINK LENGTHS
#define L1 0.08
#define L2 0.35
#define L3 0.33

void forward_kinematics(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs forward kinematics for a 3R spatial manipulator.
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
    end_effector_pos[0] = l2*cos(thetas[1] + PI/2) + l3*cos(thetas[1] + PI/2 + thetas[2]);
    end_effector_pos[1] = (l2*sin(thetas[1] + PI/2) + l3*sin(thetas[1] + PI/2 + thetas[2]))*sin(thetas[0]);
    end_effector_pos[2] = l1 + (l2*sin(thetas[1] + PI/2) + l3*sin(thetas[1] + PI/2 + thetas[2]))*cos(thetas[0]);
}


void jacobian(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Jacobian of the end effector given a certain configuration of the robot
    thetas = [abduction, hip, knee] angles in radians
    jacob = Dq/Dthetas where q = [x,y,z]
    l1, l2, l3 = link lengths
    */
//    jacob << -(l2*cos(thetas[1])*sin(thetas[0])) - l3*cos(thetas[1] + thetas[2])*sin(thetas[0]),-(l2*cos(thetas[0])*sin(thetas[1])) - l3*cos(thetas[0])*sin(thetas[1] + thetas[2]),-(l3*cos(thetas[0])*sin(thetas[1] + thetas[2])),
//    l2*cos(thetas[0])*cos(thetas[1]) + l3*cos(thetas[0])*cos(thetas[1] + thetas[2]),-(l2*sin(thetas[0])*sin(thetas[1])) - l3*sin(thetas[0])*sin(thetas[1] + thetas[2]),-(l3*sin(thetas[0])*sin(thetas[1] + thetas[2])),
//    0,l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]),l3*cos(thetas[1] + thetas[2]);
   double theta1 = thetas[1]+PI/2;
   jacob << 0,-(l2*sin(theta1)) - l3*sin(theta1 + thetas[2]),-(l3*sin(theta1 + thetas[2])),cos(thetas[0])*(l2*sin(theta1) + l3*sin(theta1 + thetas[2])),(l2*cos(theta1) + l3*cos(theta1 + thetas[2]))*sin(thetas[0]),l3*cos(theta1 + thetas[2])*sin(thetas[0]),
   -(sin(thetas[0])*(l2*sin(theta1) + l3*sin(theta1 + thetas[2]))),cos(thetas[0])*(l2*cos(theta1) + l3*cos(theta1 + thetas[2])),l3*cos(thetas[0])*cos(theta1 + thetas[2]);
}

void inverse_dynamics(Eigen::Vector3d &torques, Eigen::Vector3d &force, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Joint torques for desired force at end effector
    torques =  [abduction, hip, knee]
    force = [x,y,z]
    thetas = [abduction, hip, knee] angles in radians
    l1, l2, l3 = link lengths
    */
    Eigen::Matrix<double,3,3> jacob;
    jacobian(jacob, thetas, l1, l2, l3);
    torques = jacob.transpose()*force; 
}  

void inverse_kinematics(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH1
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = atan2(y, z-l1);
   r = sqrt((z-l1)*(z-l1) + y*y);
   t = (-4*l3*r + sqrt(16*pow(l3,2)*pow(r,2) - 4*(-pow(l2,2) + pow(l3,2) + pow(r,2) - 2*l3*x + pow(x,2))*(-pow(l2,2) + pow(l3,2) + pow(r,2) + 2*l3*x + pow(x,2))))/
   (2.*(pow(l2,2) - pow(l3,2) - pow(r,2) - 2*l3*x - pow(x,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(r- l3*sin(theta23), x - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
   thetas[1] -= PI/2;
}





// int main()
// {
//     // Eigen::Vector3d thetas(0,0,0);

//     // for(int i=0;i<3;i++)
//     // {
//     //   std::cout<<"\n"<<"theta"<<i<<":";
//     //   std::cin>>thetas(i);
//     // }
//     // float l1 = 1.;
//     // float l2 = 1.;
//     // float l3 = 1.;
//     // Eigen::Vector3d pos;
//     // Eigen::Vector3d thetas2;
//     // //Testing inverse Kinematics
//     // forward_kinematics_2(pos, thetas);
//     // std::cout<<"\nLeg_Frame:\t"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
//     // double initial_height = 0.91;
//     // Eigen::Vector3d base(0,0,initial_height);
//     // std::cout<<"\nbase in raisim is @:\n"<<base<<std::endl;
    
//     // //std::cout<<"\nBase_Frame:\t"<<pos[2]<<" "<<pos[1]<<" "<<pos[0]+0.23<<std::endl;
//     // Eigen::Matrix<double,3,3> Rot_legframe_to_baseframe;
//     // Rot_legframe_to_baseframe << 0, 0,-1,
//     //                              0,-1, 0,
//     //                             -1, 0, 0;

//     // pos = Rot_legframe_to_baseframe*pos;
//     // std::cout<<"\nwrt_hip_frame:\t"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;

//     Eigen::Vector3d thetasr(1.3,3.1,1.6);
//     Eigen::Vector3d pos(0,0,0);
//     Eigen::Vector3d thetas(0,0,0);
//     forward_kinematics(pos, thetasr);
//     std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
//     inverse_kinematics(thetas, pos);
//     forward_kinematics(pos, thetas);
//     std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
// //     inverse_kinematics_branch2(thetas, pos, l1, l2, l3);
// //     forward_kinematics(pos, thetas);
// //     std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
// //     inverse_kinematics_branch3(thetas, pos, l1, l2, l3);
// //     forward_kinematics(pos, thetas);
// //     std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
// //     inverse_kinematics_branch4(thetas, pos, l1, l2, l3);
// //     forward_kinematics(pos, thetas);
// //     std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
// //     inverse_kinematics(thetas, pos, 1);
// //     forward_kinematics(pos, thetas);
// //     std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    
// //     //Testing Jacobian -- eqs are correct
// //     Eigen::Matrix<double, 3, 3> jacob;
// //     jacobian_2(jacob, thetas);
// //     std::cout<<std::endl<<"Jacobian:"<<std::endl;
// //     std::cout<<jacob<<std::endl;
    
// //     Eigen::Matrix<double, 3, 3> jacob_inv = (Rot_legframe_to_baseframe*jacob).inverse();

// //     Eigen::Vector3d tou(-0.926493,-0.0524843,-7.40437e-07);//0.2,0.2,-0.6
// //     std::cout<<std::endl<<"Generalized_force:"<<std::endl<<tou;

// //    std::cout<<std::endl<<"((R*J)^-1)^T * tou"<<std::endl<<jacob_inv.transpose()*tou; 

// //     //Testing Inverse Dynamics-- eqs are correct
// //     Eigen::Vector3d forces(1,1,1);
// //     Eigen::Vector3d torques;
// //     inverse_dynamics(torques, forces,thetas);
// //     std::cout<<"torques: "<<torques<<std::endl;
//     return 0;
// }