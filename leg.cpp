#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#define PI  3.141592

void forward_kinematics(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1, float l2, float l3)
{
    /*
    Performs forward kinematics for a 3R spatial manipulator.
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
    end_effector_pos[0] = cos(thetas[0])*(l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]));
    end_effector_pos[1] = (l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0]);
    end_effector_pos[2] = l1 + l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]);
}

void inverse_kinematics_branch1(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1, float l2, float l3)
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
   thetas[0] = atan2(y, x);
   r = sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z - sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}

void inverse_kinematics_branch2(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1, float l2, float l3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH2
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = atan2(y, x);
   r = sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z + sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}
void inverse_kinematics_branch3(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1, float l2, float l3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH3
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = PI + atan2(y, x);
   r = -1*sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z - sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}
void inverse_kinematics_branch4(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1, float l2, float l3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH3
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = PI + atan2(y, x);
   r = -1*sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z + sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}
void inverse_kinematics(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1, float l2, float l3, int branch)
{
    if(branch == 1)
    {
        inverse_kinematics_branch1(thetas, end_effector_pos, l1, l2, l3);
    }
    if(branch == 2)
    {
        inverse_kinematics_branch2(thetas, end_effector_pos, l1, l2, l3);
    }
    if(branch == 3)
    {
        inverse_kinematics_branch3(thetas, end_effector_pos, l1, l2, l3);
    }
    if(branch == 4)
    {
        inverse_kinematics_branch4(thetas, end_effector_pos, l1, l2, l3);
    }
}


void jacobian(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1, float l2, float l3)
{
    /*
    Finds Jacobian of the end effector given a certain configuration of the robot
    thetas = [abduction, hip, knee] angles in radians
    jacob = Dq/Dthetas where q = [x,y,z]
    l1, l2, l3 = link lengths
    */
   jacob << -(l2*cos(thetas[1])*sin(thetas[0])) - l3*cos(thetas[1] + thetas[2])*sin(thetas[0]),-(l2*cos(thetas[0])*sin(thetas[1])) - l3*cos(thetas[0])*sin(thetas[1] + thetas[2]),-(l3*cos(thetas[0])*sin(thetas[1] + thetas[2])),
   l2*cos(thetas[0])*cos(thetas[1]) + l3*cos(thetas[0])*cos(thetas[1] + thetas[2]),-(l2*sin(thetas[0])*sin(thetas[1])) - l3*sin(thetas[0])*sin(thetas[1] + thetas[2]),-(l3*sin(thetas[0])*sin(thetas[1] + thetas[2])),
   0,l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]),l3*cos(thetas[1] + thetas[2]);
}

void inverse_dynamics(Eigen::Vector3d &torques, Eigen::Vector3d &force, Eigen::Vector3d &thetas, float l1, float l2, float l3)
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

int main()
{
    Eigen::Vector3d thetas(1,0.55,1.70);
    float l1 = 1.;
    float l2 = 1.;
    float l3 = 1.;
    Eigen::Vector3d pos;
    Eigen::Vector3d thetas2;
    //Testing inverse Kinematics
    forward_kinematics(pos, thetas, l1, l2 ,l3);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics_branch1(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas, l1, l2 ,l3);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics_branch2(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas, l1, l2 ,l3);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics_branch3(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas, l1, l2 ,l3);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics_branch4(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas, l1, l2 ,l3);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics(thetas, pos, l1, l2, l3, 1);
    forward_kinematics(pos, thetas, l1, l2 ,l3);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    
    //Testing Jacobian -- eqs are correct
    Eigen::Matrix<double, 3, 3> jacob;
    jacobian(jacob, thetas, l1, l2, l3);
    std::cout<<jacob<<std::endl;

    //Testing Inverse Dynamics-- eqs are correct
    Eigen::Vector3d forces(1,1,1);
    Eigen::Vector3d torques;
    inverse_dynamics(torques, forces,thetas, l1, l2, l3);
    std::cout<<"torques: "<<torques<<std::endl;
    return 0;
}