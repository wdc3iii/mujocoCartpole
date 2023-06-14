#include "mujoco_interface.h"

// Generic Mujoco functions and variables

MujocoInterface::MujocoInterface(){}

MujocoInterface::~MujocoInterface(){}

void MujocoInterface::MujocoSetup(const char file_name[]) {
    char error_msg[1000] = "Failed to load binary model";

    MJ_MODEL_PTR = mj_loadXML(file_name, 0, error_msg, 1000);

    if(!MJ_MODEL_PTR)
    {
        mju_error_s("Load model error: %s", error_msg);
    }
    else
    {
        std::cout << "Model was loaded successfully" << std::endl;
    }

    MJ_DATA_PTR = mj_makeData(MJ_MODEL_PTR);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&MJ_CAMERA);
    mjv_defaultOption(&MJ_OPTIONS);
    mjv_defaultScene(&MJ_SCENE);
    mjr_defaultContext(&MJ_CONTEXT);
    mjv_makeScene(MJ_MODEL_PTR, &MJ_SCENE, 2000);                // space for 2000 objects
    mjr_makeContext(MJ_MODEL_PTR, &MJ_CONTEXT, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // Set defualt camera view
    double arr_view[] = {90, -5, 5, 0.012768, -0.000000, 1.254336};
    MJ_CAMERA.azimuth = arr_view[0];
    MJ_CAMERA.elevation = arr_view[1];
    MJ_CAMERA.distance = arr_view[2];
    MJ_CAMERA.lookat[0] = arr_view[3];
    MJ_CAMERA.lookat[1] = arr_view[4];
    MJ_CAMERA.lookat[2] = arr_view[5];

}

void MujocoInterface::MujocoShutdown() {
    // free visualization storage
    mjv_freeScene(&MJ_SCENE);
    mjr_freeContext(&MJ_CONTEXT);

    // free MuJoCo model and data, deactivate
    mj_deleteData(MJ_DATA_PTR);
    mj_deleteModel(MJ_MODEL_PTR);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

void MujocoInterface::UpdateScene() {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
    mjv_updateScene(MJ_MODEL_PTR, MJ_DATA_PTR, &(MJ_OPTIONS), NULL, &MJ_CAMERA, mjCAT_ALL, &MJ_SCENE);
    mjr_render(viewport, &MJ_SCENE, &MJ_CONTEXT);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(MJ_MODEL_PTR, MJ_DATA_PTR);
        mj_forward(MJ_MODEL_PTR, MJ_DATA_PTR);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    BUTTON_LEFT =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    BUTTON_MIDDLE = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    BUTTON_RIGHT =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &BUTTON_LAST_X, &BUTTON_LAST_Y);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if( !BUTTON_LEFT && !BUTTON_MIDDLE && !BUTTON_RIGHT )
        return;

    // compute mouse displacement, save
    double dx = xpos - BUTTON_LAST_X;
    double dy = ypos - BUTTON_LAST_Y;
    BUTTON_LAST_X = xpos;
    BUTTON_LAST_Y = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( BUTTON_RIGHT )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( BUTTON_LEFT )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(MJ_MODEL_PTR, action, dx/height, dy/height, &MJ_SCENE, &MJ_CAMERA);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(MJ_MODEL_PTR, mjMOUSE_ZOOM, 0, -0.05*yoffset, &MJ_SCENE, &MJ_CAMERA);
}


// Robot specific functions

Vector4d MujocoInterface::getState() {
    Vector4d state;

    // [x, theta, xdot, thetadot]
    state(0) = MJ_DATA_PTR->qpos[0];
    state(1) = MJ_DATA_PTR->qpos[1];
    state(2) = MJ_DATA_PTR->qvel[0];
    state(3) = MJ_DATA_PTR->qvel[1]; 

    return state;
}


void MujocoInterface::setState(Vector4d state) {
    // Base position
    MJ_DATA_PTR->qpos[0] = state(0);
    MJ_DATA_PTR->qpos[1] = state(1);
    MJ_DATA_PTR->qvel[0] = state(2);
    MJ_DATA_PTR->qvel[1] = state(3);
}


void MujocoInterface::commandForce(double force) {
    MJ_DATA_PTR->ctrl[0] = force;
}
