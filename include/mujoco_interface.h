#ifndef mujoco_interface_h
#define mujoco_interface_h

#include "stdio.h"
#include <iostream>

#include "mujoco.h"
#include "GLFW/glfw3.h"

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 4, 1> Vector4d;

// Mujoco pointers

// Mujoco model pointer
inline mjModel *MJ_MODEL_PTR = NULL;

// Mujoco data pointer
inline mjData *MJ_DATA_PTR = NULL;

// Mujoco contact pointer
inline mjContact *MJ_CONTACT_PTR = NULL;


// Mujoco data structures

// Mujoco camera
inline mjvCamera MJ_CAMERA;

// Mujoco visualization options 
inline mjvOption MJ_OPTIONS;

// Mujoco scene
inline mjvScene MJ_SCENE;

// Mujoco GPU options
inline mjrContext MJ_CONTEXT;


// Mujoco mouse interactions

// Left mouse click
inline bool BUTTON_LEFT = false;

// Middle mouse click
inline bool BUTTON_MIDDLE = false;

// Right mouse click
inline bool BUTTON_RIGHT =  false;

// X position of last mouse click
inline double BUTTON_LAST_X = 0;

// Y position of last mouse click
inline double BUTTON_LAST_Y = 0;


// Mujoco interface class
class MujocoInterface
{
    // Generic Mujoco functions and variables

    /// \brief Constructor
    public: MujocoInterface();

    /// \brief Destructor
    public: virtual ~MujocoInterface();

    /// \brief Pointer to the Mujoco graphics window
    public: GLFWwindow* window;

    /// \brief The MujocoSetup function initializes the simulator
    /// by loading the model and setting up the data interface, 
    /// physics, scene, graphics, camera, and user interaction. 
    public: void MujocoSetup(const char file_name[]);

    /// \brief The MujocoShutdown function shuts down the simulator
    /// and releases all the resources allocated by it. 
    public: void MujocoShutdown();

    /// \brief The UpdateScene function updates the visuals of
    /// of the mujoco simulator
    public: void UpdateScene();


    // Robot specific functions

    /// \brief The GetState function gets the latest cartpole state
    /// \return Returns a vector containing the state of the robot
    /// in the world frame where the orientation is given in ZYX Euler coordinates
    /// (x, theta, xdot, thetadot)
    public: Vector4d getState();

    /// \brief The SetState function initializes all of the position and velocity states
    /// of the robot in the simulation environment to the ones specified by the inputs
    /// \param[in] state The current state of the robot 
    public: void setState(Vector4d state);

    /// \brief Send a joint torque reference to the robots actuators
    /// \param[in] force the commanded force on the cart
    public: void commandForce(double force);
};

// Simulation interaction callback functions (DO NOT CHANGE)

/// \brief The keyboard function is the callback function for keyboard presses
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

/// \brief The mouse_button function is the callback function for mouse button presses
void mouse_button(GLFWwindow* window, int button, int act, int mods);

/// \brief The mouse_move function is the callback function for mouse movements
void mouse_move(GLFWwindow* window, double xpos, double ypos);

/// \brief The scroll function is the callback function for mouse scrolling presses
void scroll(GLFWwindow* window, double xoffset, double yoffset);

#endif