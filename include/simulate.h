#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static bool glfw_initialized;
static int fontscale;

struct cassie_vis {
    //visual interaction controls
    double lastx;
    double lasty;
    bool button_left;
    bool button_middle;
    bool button_right;

    int lastbutton;
    double lastclicktm;

    int refreshrate;

    int showhelp;
    bool showoption;
    bool showdepth;
    bool showfullscreen;
    bool showsensor;
    bool slowmotion;

    bool showinfo;
    bool paused;

    int framenum;
    int lastframenum;
    
    // GLFW  handle
    GLFWwindow *window;

    // MuJoCo stuff

    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvPerturb pert;
    mjvFigure figsensor;
    mjModel* m;
    mjData* d;
};
typedef struct cassie_vis cassie_vis_t;

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* w, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);
void window_close_callback(GLFWwindow *window);
void update_scene(cassie_vis_t* v);
void render(cassie_vis_t* v);
cassie_vis_t* cassie_vis_init(mjModel* m, mjData* d);