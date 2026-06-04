#include <stdio.h>
#include <stdlib.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "arm_core/joint_sweep.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/sim_loop.h"

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

int main(int argc, char **argv) {
  const char *model_path = argc > 1 ? argv[1] : default_model_path();

  char load_error[1024] = {0};
  mjModel *model = mj_loadXML(model_path, NULL, load_error, sizeof(load_error));
  if (!model) {
    fprintf(stderr, "Failed to load model '%s': %s\n", model_path, load_error);
    return EXIT_FAILURE;
  }

  mjData *data = mj_makeData(model);
  if (!data) {
    fprintf(stderr, "Failed to allocate MuJoCo data.\n");
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  arm_config_t config = armsim_default_arm6_config();
  mujoco_arm_t arm;
  char bind_error[256] = {0};
  if (mujoco_arm_bind(model, &config, &arm, bind_error, sizeof(bind_error)) != ARM_OK) {
    fprintf(stderr, "Failed to bind arm: %s\n", bind_error);
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  GLFWwindow *window = glfwCreateWindow(1280, 720, "ArmSim MuJoCo Viewer", NULL, NULL);
  if (!window) {
    fprintf(stderr, "Failed to create GLFW window.\n");
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjvCamera camera;
  mjvOption option;
  mjvScene scene;
  mjrContext context;
  mjv_defaultCamera(&camera);
  mjv_defaultOption(&option);
  mjv_defaultScene(&scene);
  mjr_defaultContext(&context);
  mjv_makeScene(model, &scene, 2000);
  mjr_makeContext(model, &context, mjFONTSCALE_150);

  camera.distance = 1.8;
  camera.azimuth = 135.0;
  camera.elevation = -25.0;

  joint_sweep_t sweep;
  joint_sweep_init(&sweep, config.dof, (joint_sweep_params_t){1.0, 0.8, 0.5});
  arm_controller_t controller = joint_sweep_as_controller(&sweep);
  arm_state_t state;
  arm_command_t command;

  while (!glfwWindowShouldClose(window)) {
    const mjtNum frame_start = data->time;
    while (data->time - frame_start < 1.0 / 60.0 && !sweep.complete) {
      if (armsim_step_once(model, data, &arm, &config, &controller, &state, &command) != ARM_OK) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        break;
      }
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    const mjrRect viewport = {0, 0, width, height};
    mjv_updateScene(model, data, &option, NULL, &camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjr_freeContext(&context);
  mjv_freeScene(&scene);
  glfwDestroyWindow(window);
  glfwTerminate();
  mj_deleteData(data);
  mj_deleteModel(model);
  return EXIT_SUCCESS;
}
