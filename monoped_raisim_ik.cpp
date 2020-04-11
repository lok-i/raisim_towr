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

/*
This code works the inverse kinematics 
for the monoped and updates it in a gravity free 
environment were the base (pd gains are true 
for all values) is fixed and only 
the angles are updated.

*/

#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include <xpp_inv/hyqleg_inverse_kinematics.h>
#include <xpp_inv/cartesian_declarations.h>




//converts end effector from ground reference to base refernce such
//that axes are parallel
Eigen::Vector3d grnd_ref_to_base_ref(Eigen::Vector3d ee,Eigen::Vector3d base)
{
 Eigen::Vector3d offset_base_to_hip(0.0, 0.0, 0.15);
 base = base -offset_base_to_hip;//base to hip offset

 return(ee-base);
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

int main(int argc, char **argv) {
  /// create raisim world
  xpp::HyqlegInverseKinematics leg;
  raisim::World world;
  
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
  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto ground = world.addGround();
  

  /// create visualizer objects
  auto groundVis = vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");



 

  
  //no of steps per call
  const size_t N = 100;

  auto monoped = world.addArticulatedSystem(raisim::loadResource("monoped/monoped.urdf"));
  auto monopedVis = vis->createGraphicalObject(monoped, "monoped");
  
  monoped->setGeneralizedCoordinate({   0, 0, 1, //base co ordinates 
                                        1, 0, 0, 0,  //orientation 
                                        0,0,0});

//-3.14,-1.57,-2.91

  Eigen::VectorXd jointNominalConfig(monoped->getDOF()+1), jointVelocityTarget(monoped->getDOF());
  Eigen::VectorXd jointState(monoped->getDOF()), jointForce(monoped->getDOF()), jointPgain(monoped->getDOF()), jointDgain(monoped->getDOF());
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  

 // P and D gains for the leg actuators alone
 jointPgain.tail(9).setConstant(200.0);
 jointDgain.tail(9).setConstant(10.0);


  monoped->setGeneralizedForce(Eigen::VectorXd::Zero(monoped->getDOF()));
  monoped->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  monoped->setPdGains(jointPgain, jointDgain);
  monoped->setName("monoped");
  
  //to take random samples
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.7);
  std::srand(std::time(nullptr));
 // monoped->printOutBodyNamesInOrder();
 


  // lambda function for the controller
  auto controller = [&monoped ,&generator, &distribution,&leg]() {
    static size_t controlDecimation = 0;

    if (controlDecimation++ % 2500 == 0)
      monoped->setGeneralizedCoordinate({0, 0, 1,

                                        1, 0, 0,0, 

                                        0,0,0});

    if (controlDecimation % 50 != 0)
    return;

    /// laikago joint PD controller
    Eigen::VectorXd jointNominalConfig(monoped->getDOF()+1), jointVelocityTarget(monoped->getDOF());
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 1, 
                          1, 0, 0, 0, 
                          0, 0, 0;


Eigen::Vector3d Base(0,0,1);
Eigen::Vector3d ee_grnd(0.1,0.1,0.6); //end effector wrt ground


Eigen::Vector3d ee_H = grnd_ref_to_base_ref(ee_grnd,Base);//end effector wrt base
Eigen::VectorXd q0 =leg.GetJointAngles(ee_H);




  for (size_t i = 0; i < N; i++) 
      
      for (size_t j = 0; j < N; j++) {
 
          jointNominalConfig << 0, 0, 0.5, -1, 0, 0, 0, 0,0 ,0;

        for (size_t k = 0; k < monoped->getGeneralizedCoordinateDim() ; k++)
        {
        
         if(k>=7)
          jointNominalConfig(k)=q0(k-7);//distribution(generator);


         }
       
      
        monoped->setPdTarget(jointNominalConfig, jointVelocityTarget);
    
      }
    

    



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
