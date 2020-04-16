
//
// Created by Jemin Hwangbo on 10/15/10.
// MIT License
//
// Copyright (c) 2010-2010 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include <xpp_inv/hyqleg_inverse_kinematics.h>
#include <xpp_inv/cartesian_declarations.h>


#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <typeinfo>




using namespace towr;
#define base_height_initial 0.91

#define gravity true
#define actuators_only false
#define PD_tuning_mode false



//converts end effector from ground reference to base refernce such
//that axes are parallel
Eigen::Vector3d grnd_ref_to_base_ref(Eigen::Vector3d ee,Eigen::Vector3d base)
{
 Eigen::Vector3d offset_base_to_hip(0.0, 0.0, 0.15);
 base = base -offset_base_to_hip;//base to hip offset

 return(ee-base);
}

//trajectory follower for ik testing
Eigen::Vector3d fn_modified_sine(float time , float amp = 0.05,float omega = 10)
  {
    Eigen::Vector3d end_effector_pos(amp*sin(time*omega), 0., amp*cos(time*omega) - 0.5);
    return(end_effector_pos);
  }

void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3, -0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shdow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(30);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.06, .6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}

int main(int argc, char **argv) 
{

std::ios_base::sync_with_stdio(false);
std::cin.tie(0);
std::cout.tie(0);
float P_gain = 200.0,D_gain =10.0;
if(PD_tuning_mode)
  {
  std::cout<<"Enter P and D gains:"<<std::endl;
  std::cin>>P_gain>>D_gain;
  }

xpp::HyqlegInverseKinematics leg;
raisim::World world;
if(!gravity)
  world.setGravity({0,0,0}); // by default gravity is set to {0,0,g}
world.setTimeStep(0.0025);

  auto vis = raisim::OgreVis::get();

  /// these method must be called before initApp
  vis->setWorld(&world);
  vis->setWindowSize(2600, 1200);
  vis->setImguiSetupCallback(imguiSetupCallback);
  vis->setImguiRenderCallback(imguiRenderCallBack);
  vis->setKeyboardCallback(raisimKeyboardCallback);
  vis->setSetUpCallback(setupCallback);
  vis->setAntiAliasing(2);
  vis->setDesiredFPS(25);

  //simulation is automatically stepped, if is false
  raisim::gui::manualStepping = false; 
  //raisim::gui::Collisionbodies = true; 
  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto ground = world.addGround();
  

  /// create visualizer objects
  auto groundVis = vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

   
 



  auto monoped = world.addArticulatedSystem(raisim::loadResource("monoped/monoped.urdf"));
  auto monopedVis = vis->createGraphicalObject(monoped, "monoped");
  
  monoped->setGeneralizedCoordinate({   0, 0, base_height_initial, //base co ordinates 
                                        1, 0, 0, 0,  //orientation 
                                        0,1.09542,-2.3269});




  Eigen::VectorXd jointNominalConfig(monoped->getDOF()+1), jointVelocityTarget(monoped->getDOF());
  Eigen::VectorXd jointState(monoped->getDOF()), jointForce(monoped->getDOF()), jointPgain(monoped->getDOF()), jointDgain(monoped->getDOF());
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  
if (actuators_only)
  
  {
    jointPgain.tail(3).setConstant(P_gain);
   jointDgain.tail(3).setConstant(D_gain);}

  else
  {

     for(int k =0;k<9;k++)
     {
      if(k<=2)//for base linear posn
      {
        jointPgain(k)=P_gain;
        jointDgain(k)=D_gain;

      }
      else
      {

        if(k<=5) //for base orientation
        {
          jointPgain(k)=2000.0;
          jointDgain(k)=100.0;

        }
        else //for actuators
        {
        jointPgain(k)=P_gain;
        jointDgain(k)=D_gain;
        }

     
      }}}


  //monoped->setGeneralizedForce(Eigen::VectorXd::Zero(monoped->getDOF()));
  monoped->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  monoped->setPdGains(jointPgain, jointDgain);
  monoped->setName("monoped");
  
  //to take random samples
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.7);
  std::srand(std::time(nullptr));
  monoped->printOutBodyNamesInOrder();
  double t = 0.0;
 


  // lambda function for the controller
  auto controller = [&monoped ,&generator, &distribution,&leg,&t,&world]()
   {
    static size_t controlDecimation = 0;

    if (controlDecimation++ % 10000 == 0)
      {//std::cout<<"Reset"<<controlDecimation<<std::endl;
      monoped->setGeneralizedCoordinate({0, 0, base_height_initial,

                                        1, 0, 0,0, 

                                         0,1.09542,-2.3269});
    }

    if (controlDecimation % 50 != 0)
    {//std::cout<<"halt"<<controlDecimation<<std::endl;
      return;
      }
    /// laikago joint PD controller
    Eigen::VectorXd jointNominalConfig(monoped->getDOF()+1), jointVelocityTarget(monoped->getDOF());
    jointVelocityTarget.setZero();
    
    jointNominalConfig << 0, 0, base_height_initial, 
                          1, 0, 0, 0, 
                          0,1.09542,-2.3269;


   




        Eigen::Vector3d Base(0,0,base_height_initial);
        Eigen::Vector3d ee_grnd;//={0.2,0.2,0.2};//fn_modified_sine(t,0.9,100); //end effector wrt ground 
        

        std::cout<<"co_ordianates"<<"\t";
        for (int j =0;j<3;j++)
        std::cin>>ee_grnd(j);

        std::cout<<"ee_grnd:"<<ee_grnd<<std::endl;
        Eigen::Vector3d ee_H = grnd_ref_to_base_ref(ee_grnd,Base);//end effector wrt base
        std::cout<<"ee_H:"<<ee_H<<std::endl;
        Eigen::Vector3d q0 =leg.GetJointAngles(ee_H);
        std::cout<<"q0:\n"<<q0<<std::endl;
       // monoped->setGeneralizedCoordinate({0,0,base_height_initial,1,0,0,0,q0[0],q0[1],q0[2]});
         t+= world.getTimeStep();

        for (size_t k = 0; k < monoped->getGeneralizedCoordinateDim() ; k++)
        {
         
         //if(k<=2)
         //jointNominalConfig(k) = Base(k);
         if(k>=7)
         jointNominalConfig(k) = q0(k-7);//distribution(generator);

         }
       
        //std::cout<<"jointNominalConfig"<<jointNominalConfig<<std::endl;
       monoped->setPdTarget(jointNominalConfig, jointVelocityTarget);

      //print contact force details..

        /*  auto footIndex = monoped->getBodyIdx("lowerleg");

      /// for all contacts on the robot, check ...
      for(auto& contact: monoped->getContacts())
       {
        if ( footIndex == contact.getlocalBodyIndex() ) {
        //std::cout<<"Contact impulse in the contact frame: "<<contact.getImpulse()->e()<<std::endl;
         // std::cout<<"Contact frame: \n"<<contact.getContactFrame().e()<<std::endl;
          std::cout<<"Contact impulse in the world frame: "<<contact.getContactFrame().e() * contact.getImpulse()->e()<<std::endl;
          std::cout<<"Contact Normal in the world frame: "<<contact.getNormal().e().transpose()<<std::endl;
          std::cout<<"Contact position in the world frame: "<<contact.getPosition().e().transpose()<<std::endl;
         // std::cout<<"It collides with: "<<world.getObject(contact.getPairObjectIndex().getName())<<std::endl;
          std::cout<<std::endl<<std::endl;
        }
      }
*/        std::cout<<std::endl;
       // for(auto& bodyName: monoped->getBodyNames())
       // {
         
        
       // }
          




          raisim::SparseJacobian sparseJaco;
          raisim::Vec<3> point_W = {0,0,0};
          auto bodyIndex = monoped->getBodyIdx("lowerleg");



          monoped->getSparseJacobian(bodyIndex,point_W,sparseJaco);
  
          //std::cout<<bodyName<<":";
          std::cout<<"Sparse_Jacobian:"<<std::endl;
          std::cout<< sparseJaco.v<<"\t";
          std::cout<<std::endl;
          std::cout<<"Generalized Velocity:"<<monoped->getGeneralizedVelocity()<<std::endl;
          std::cout<<"Generalized Force:"<<monoped->getGeneralizedForce()<<std::endl;
          std::cout<<"Generalized Feedforward Force:"<<monoped->getFeedForwardGeneralizedForce()<<std::endl;

      

  };

 vis->setControlCallback(controller);

  /// set camera
  vis->select(groundVis->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);

  /// run the app

  vis->run();

  /// terminate
  vis->closeApp();



  return 0;
}