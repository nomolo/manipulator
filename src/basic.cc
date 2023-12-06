#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>

#include "controller/controller.h"
#include "display_data.h"

// MuJoCo data structures
mjModel* m = NULL;  // MuJoCo model
mjData* d = NULL;   // MuJoCo data
mjvCamera cam;      // abstract camera
mjvOption opt;      // visualization options
mjvScene scn;       // abstract scene
mjrContext con;     // custom GPU context

// Controller
mnplt::Controller ctrler;

// set
double stiffness = 100.0;
double motor_inertias = 1.0;

// Data diaplay
// mnplt::DataDisplay disp;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// control callback
void control_callback(const mjModel* m, mjData* d) { ctrler.control(m, d); }

// actuator dynamic callback
mjtNum act_dyn_callback(const mjModel* m, const mjData* d, int id) {
  if (m->actuator_actnum[id] != 2) {
    mju_error("callback expected actnum == 2");
  }

  // get pointers to activations (inputs) and their derivatives (outputs)
  mjtNum* act = d->act + m->actuator_actadr[id];
  mjtNum* act_dot = d->act_dot + m->actuator_actadr[id];

  // get joint's id for actuator
  auto joint_id = m->actuator_trnid[id * 2];

  // harmonic oscillator with controlled frequency
  act_dot[0] =
      (d->ctrl[id] + stiffness * (d->qpos[m->jnt_qposadr[joint_id]] - act[1])) /
      motor_inertias;
  act_dot[1] = act[0];

  return 0;  // ignored by caller
}

//
mjtNum act_bias_callback(const mjModel* m, const mjData* d, int id) {
  if (m->actuator_actnum[id] != 2) {
    mju_error("callback expected actnum == 2");
  }

  mjtNum* act = d->act + m->actuator_actadr[id];
  // get joint's id for actuator
  auto joint_id = m->actuator_trnid[id * 2];

  return -stiffness * (d->qpos[m->jnt_qposadr[joint_id]] - act[1]);
}

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc != 2) {
    std::printf(" USAGE:  basic modelfile\n");
    return 0;
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(argv[1]) > 4 &&
      !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // set controller
  mjcb_control = control_callback;

  // config actuator's dynamic and bias
  mjcb_act_dyn = act_dyn_callback;
  mjcb_act_bias = act_bias_callback;

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "simulate", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.9, &scn, &cam);
  // mjv_moveCamera(m, mjMOUSE_MOVE_H, 80, 800, &scn, &cam);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // std::cout << "=====nq" << m->nq << std::endl;
  // std::cout << "=====nv" << m->nv << std::endl;
  // std::cout << "=====nu" << m->nu << std::endl;
  // std::cout << "=====na" << m->na << std::endl;

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually
    //  can, this loop will finish on time for the next frame to be rendered
    //  at 60 fps. Otherwise add a cpu timer and exit this loop when it is
    //  time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 100.0) {
      // //================================
      // //================================Debug print
      // mjtNum* act = d->act + m->actuator_actadr[2];
      // mjtNum* act_dot = d->act_dot + m->actuator_actadr[2];
      // auto joint_id = m->actuator_trnid[2 * 2];
      // std::cout << "--------------------------" << std::endl;
      // std::cout << "id:" << joint_id << std::endl;
      // // std::cout << "ctrl1:" << d->ctrl[1] << std::endl;
      // std::cout << "ctrl2:" << d->ctrl[2] << std::endl;
      // // std::cout << "force1:" << d->actuator_force[1] << std::endl;
      // std::cout << "force2:" << d->actuator_force[2] << std::endl;
      // // std::cout << "act0:" << act[0] << std::endl;
      // std::cout << "act1:" << act[1] << std::endl;
      // std::cout << "time2" << d->time << std::endl;
      // std::cout << "qpos:" << d->qpos[m->jnt_qposadr[joint_id]] << std::endl;
      // std::cout << "force:"
      //           << -stiffness * (d->qpos[m->jnt_qposadr[joint_id]] - act[1])
      //           << std::endl;
      // //================================
      // //================================

      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}
