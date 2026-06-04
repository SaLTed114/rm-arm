#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "arm_core/joint_sweep.h"
#include "armsim/default_arm_config.h"
#include "armsim/mujoco_arm.h"
#include "armsim/sim_loop.h"

#define PANEL_WIDTH 340
#define SLIDER_X 24
#define SLIDER_W 220
#define SLIDER_H 18
#define ROW_H 58

typedef struct {
  mjModel *model;
  mjData *data;
  mujoco_arm_t arm;

  mjvCamera camera;
  mjvOption option;
  mjvScene scene;
  mjrContext context;

  arm_t core;
  joint_sweep_t sweep;
  arm_controller_t controller;

  double joint_angle_rad[ARM_DOF_MAX];
  bool sweep_running;
  int active_slider;

  bool button_left;
  bool button_middle;
  bool button_right;
  double last_x;
  double last_y;
} viewer_app_t;

static const char *default_model_path(void) {
  return "sim_mujoco/models/arm6_placeholder.xml";
}

static double clamp_double(double value, double min_value, double max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void cursor_to_framebuffer(GLFWwindow *window, double cursor_x, double cursor_y, double *fb_x, double *fb_y) {
  int win_w = 1;
  int win_h = 1;
  int fb_w = 1;
  int fb_h = 1;
  glfwGetWindowSize(window, &win_w, &win_h);
  glfwGetFramebufferSize(window, &fb_w, &fb_h);

  *fb_x = cursor_x * (double)fb_w / (double)win_w;
  *fb_y = ((double)win_h - cursor_y) * (double)fb_h / (double)win_h;
}

static void joint_limits(const viewer_app_t *app, uint8_t joint, double *low, double *high) {
  const int joint_id = app->arm.joint_ids[joint];
  const arm_joint_config_t *cfg = &app->core.config.joints[joint];

  double lo = -3.141592653589793;
  double hi = +3.141592653589793;
  if (app->model->jnt_limited[joint_id]) {
    lo = app->model->jnt_range[2 * joint_id + 0];
    hi = app->model->jnt_range[2 * joint_id + 1];
  }

  const double core_lo = (double)cfg->sign * (lo - (double)cfg->q_offset_rad);
  const double core_hi = (double)cfg->sign * (hi - (double)cfg->q_offset_rad);
  *low = core_lo < core_hi ? core_lo : core_hi;
  *high = core_lo < core_hi ? core_hi : core_lo;
}

static void set_joint_angle(viewer_app_t *app, uint8_t joint, double angle_rad) {
  double low = 0.0;
  double high = 0.0;
  joint_limits(app, joint, &low, &high);

  const arm_joint_config_t *cfg = &app->core.config.joints[joint];
  const double clamped = clamp_double(angle_rad, low, high);
  app->joint_angle_rad[joint] = clamped;
  app->data->qpos[app->arm.joint_qpos_addr[joint]] =
      (mjtNum)((double)cfg->q_offset_rad + (double)cfg->sign * clamped);

  for (int i = 0; i < app->model->nv; ++i) {
    app->data->qvel[i] = 0.0;
  }
  mj_forward(app->model, app->data);
}

static void reset_manual_pose(viewer_app_t *app) {
  mj_resetData(app->model, app->data);
  for (uint8_t i = 0; i < app->core.config.dof; ++i) {
    set_joint_angle(app, i, 0.0);
  }
  joint_sweep_reset(&app->sweep);
  app->sweep_running = false;
}

static int slider_hit(const viewer_app_t *app, double fb_x, double fb_y, int fb_h) {
  (void)app;
  for (uint8_t i = 0; i < ARM_DEFAULT_DOF; ++i) {
    const int y = fb_h - 180 - (int)i * ROW_H;
    if (fb_x >= SLIDER_X && fb_x <= SLIDER_X + SLIDER_W && fb_y >= y && fb_y <= y + SLIDER_H) {
      return (int)i;
    }
  }
  return -1;
}

static void update_slider_from_mouse(viewer_app_t *app, int slider, double fb_x) {
  if (slider < 0 || slider >= app->core.config.dof) {
    return;
  }

  double low = 0.0;
  double high = 0.0;
  joint_limits(app, (uint8_t)slider, &low, &high);
  const double t = clamp_double((fb_x - SLIDER_X) / (double)SLIDER_W, 0.0, 1.0);
  set_joint_angle(app, (uint8_t)slider, low + t * (high - low));
}

static bool button_hit(double fb_x, double fb_y, int x, int y, int w, int h) {
  return fb_x >= x && fb_x <= x + w && fb_y >= y && fb_y <= y + h;
}

static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
  (void)mods;
  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }

  double cursor_x = 0.0;
  double cursor_y = 0.0;
  double fb_x = 0.0;
  double fb_y = 0.0;
  int fb_w = 1;
  int fb_h = 1;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);
  glfwGetFramebufferSize(window, &fb_w, &fb_h);
  cursor_to_framebuffer(window, cursor_x, cursor_y, &fb_x, &fb_y);

  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && fb_x < PANEL_WIDTH) {
    const int reset_y = fb_h - 106;
    const int sweep_y = fb_h - 142;
    if (button_hit(fb_x, fb_y, 24, reset_y, 132, 28)) {
      reset_manual_pose(app);
      return;
    }
    if (button_hit(fb_x, fb_y, 168, sweep_y, 132, 28)) {
      app->sweep_running = !app->sweep_running;
      if (app->sweep_running) {
        mj_resetData(app->model, app->data);
        joint_sweep_reset(&app->sweep);
      }
      return;
    }

    app->active_slider = slider_hit(app, fb_x, fb_y, fb_h);
    if (app->active_slider >= 0) {
      app->sweep_running = false;
      update_slider_from_mouse(app, app->active_slider, fb_x);
      return;
    }
  }

  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    app->button_left = action == GLFW_PRESS;
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
    app->button_middle = action == GLFW_PRESS;
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    app->button_right = action == GLFW_PRESS;
  }

  if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT) {
    app->active_slider = -1;
  }

  app->last_x = cursor_x;
  app->last_y = cursor_y;
}

static void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos) {
  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }

  double fb_x = 0.0;
  double fb_y = 0.0;
  cursor_to_framebuffer(window, xpos, ypos, &fb_x, &fb_y);
  if (app->active_slider >= 0) {
    update_slider_from_mouse(app, app->active_slider, fb_x);
    return;
  }

  if (!app->button_left && !app->button_middle && !app->button_right) {
    app->last_x = xpos;
    app->last_y = ypos;
    return;
  }

  int width = 1;
  int height = 1;
  glfwGetFramebufferSize(window, &width, &height);
  const double dx = xpos - app->last_x;
  const double dy = ypos - app->last_y;
  app->last_x = xpos;
  app->last_y = ypos;

  int action = mjMOUSE_NONE;
  if (app->button_right) {
    action = mjMOUSE_MOVE_H;
  } else if (app->button_middle) {
    action = mjMOUSE_ZOOM;
  } else if (app->button_left) {
    action = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS
                 ? mjMOUSE_MOVE_V
                 : mjMOUSE_ROTATE_H;
  }

  if (action != mjMOUSE_NONE) {
    mjv_moveCamera(app->model, action, dx / (double)height, dy / (double)height, &app->scene, &app->camera);
  }
}

static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  (void)xoffset;
  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }
  mjv_moveCamera(app->model, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &app->scene, &app->camera);
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  (void)scancode;
  (void)mods;
  if (action != GLFW_PRESS) {
    return;
  }

  viewer_app_t *app = (viewer_app_t *)glfwGetWindowUserPointer(window);
  if (!app) {
    return;
  }

  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  } else if (key == GLFW_KEY_R) {
    reset_manual_pose(app);
  } else if (key == GLFW_KEY_SPACE) {
    app->sweep_running = !app->sweep_running;
    if (app->sweep_running) {
      mj_resetData(app->model, app->data);
      joint_sweep_reset(&app->sweep);
    }
  }
}

static void draw_button(const mjrContext *context, int x, int y, int w, int h, const char *text, bool active) {
  const mjrRect rect = {x, y, w, h};
  if (active) {
    mjr_label(rect, mjFONT_NORMAL, text, 0.18f, 0.42f, 0.58f, 1.0f, 0.95f, 0.95f, 0.95f, context);
  } else {
    mjr_label(rect, mjFONT_NORMAL, text, 0.28f, 0.31f, 0.35f, 1.0f, 0.90f, 0.90f, 0.90f, context);
  }
}

static void draw_slider(viewer_app_t *app, const mjrContext *context, int fb_h, uint8_t joint) {
  double low = 0.0;
  double high = 0.0;
  joint_limits(app, joint, &low, &high);

  const int y = fb_h - 180 - (int)joint * ROW_H;
  const double value = app->joint_angle_rad[joint];
  const double t = (value - low) / (high - low);
  const int knob_x = SLIDER_X + (int)(t * (double)SLIDER_W) - 5;

  char label[80];
  char value_label[48];
  (void)snprintf(label, sizeof(label), "J%u  %.1f deg", (unsigned)(joint + 1u), value * 57.29577951308232);
  (void)snprintf(value_label, sizeof(value_label), "%.0f .. %.0f", low * 57.29577951308232, high * 57.29577951308232);

  const mjrRect label_rect = {SLIDER_X, y + 23, 128, 20};
  const mjrRect range_rect = {SLIDER_X + 146, y + 23, 100, 20};
  const mjrRect bar_rect = {SLIDER_X, y, SLIDER_W, SLIDER_H};
  const mjrRect fill_rect = {SLIDER_X, y, (int)(t * (double)SLIDER_W), SLIDER_H};
  const mjrRect knob_rect = {knob_x, y - 3, 10, SLIDER_H + 6};

  mjr_label(label_rect, mjFONT_NORMAL, label, 0.12f, 0.13f, 0.15f, 0.0f, 0.92f, 0.92f, 0.92f, context);
  mjr_label(range_rect, mjFONT_NORMAL, value_label, 0.12f, 0.13f, 0.15f, 0.0f, 0.65f, 0.68f, 0.72f, context);
  mjr_rectangle(bar_rect, 0.18f, 0.20f, 0.23f, 1.0f);
  mjr_rectangle(fill_rect, 0.25f, 0.52f, 0.68f, 1.0f);
  mjr_rectangle(knob_rect, 0.88f, 0.90f, 0.92f, 1.0f);
}

static void draw_panel(viewer_app_t *app, mjrRect viewport) {
  const mjrRect panel = {0, 0, PANEL_WIDTH, viewport.height};
  mjr_rectangle(panel, 0.08f, 0.09f, 0.10f, 0.94f);

  const mjrRect title = {20, viewport.height - 52, 260, 28};
  mjr_label(title, mjFONT_BIG, "ArmSim Pose", 0.08f, 0.09f, 0.10f, 0.0f, 0.95f, 0.95f, 0.95f, &app->context);

  draw_button(&app->context, 24, viewport.height - 106, 132, 28, "Reset", false);
  draw_button(&app->context, 168, viewport.height - 142, 132, 28, app->sweep_running ? "Stop sweep" : "Run sweep",
              app->sweep_running);

  const mjrRect hint = {24, viewport.height - 150, 260, 20};
  mjr_label(hint, mjFONT_NORMAL, "Drag sliders. Mouse scene: rotate, right-pan, wheel-zoom.", 0.08f, 0.09f, 0.10f, 0.0f,
            0.66f, 0.69f, 0.72f, &app->context);

  for (uint8_t i = 0; i < app->core.config.dof; ++i) {
    draw_slider(app, &app->context, viewport.height, i);
  }
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

  viewer_app_t app = {0};
  app.model = model;
  app.data = data;
  arm_config_t config = armsim_default_arm6_config();
  if (arm_init(&app.core, &config) != ARM_OK) {
    fprintf(stderr, "Failed to initialize arm core.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return EXIT_FAILURE;
  }
  app.active_slider = -1;

  char bind_error[256] = {0};
  if (mujoco_arm_bind(model, &app.core.config, &app.arm, bind_error, sizeof(bind_error)) != ARM_OK) {
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
  glfwSetWindowUserPointer(window, &app);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_pos_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetKeyCallback(window, key_callback);

  mjv_defaultCamera(&app.camera);
  mjv_defaultOption(&app.option);
  mjv_defaultScene(&app.scene);
  mjr_defaultContext(&app.context);
  mjv_makeScene(model, &app.scene, 2000);
  mjr_makeContext(model, &app.context, mjFONTSCALE_150);

  app.camera.distance = 1.35;
  app.camera.azimuth = 135.0;
  app.camera.elevation = -22.0;
  app.camera.lookat[0] = 0.30;
  app.camera.lookat[1] = 0.0;
  app.camera.lookat[2] = 0.20;

  joint_sweep_init(&app.sweep, app.core.config.dof, (joint_sweep_params_t){1.0, 0.8, 0.5});
  app.controller = joint_sweep_as_controller(&app.sweep);
  reset_manual_pose(&app);

  while (!glfwWindowShouldClose(window)) {
    if (app.sweep_running && !app.sweep.complete) {
      const mjtNum frame_start = data->time;
      while (data->time - frame_start < 1.0 / 60.0 && !app.sweep.complete) {
        if (armsim_step_once(model, data, &app.arm, &app.core, &app.controller) != ARM_OK) {
          glfwSetWindowShouldClose(window, GLFW_TRUE);
          break;
        }
      }
    } else {
      app.sweep_running = false;
      mj_forward(model, data);
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    const mjrRect viewport = {0, 0, width, height};
    const mjrRect scene_viewport = {PANEL_WIDTH, 0, width - PANEL_WIDTH, height};

    mjv_updateScene(model, data, &app.option, NULL, &app.camera, mjCAT_ALL, &app.scene);
    mjr_render(scene_viewport, &app.scene, &app.context);
    draw_panel(&app, viewport);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjr_freeContext(&app.context);
  mjv_freeScene(&app.scene);
  glfwDestroyWindow(window);
  glfwTerminate();
  mj_deleteData(data);
  mj_deleteModel(model);
  return EXIT_SUCCESS;
}
