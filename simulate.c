#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "simulate.h"
#include <stdbool.h>
#include <stdlib.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <dlfcn.h>
#include <libgen.h>

// bool glfw_initialized = false;
// static int fontscale = mjFONTSCALE_200;

// keyboard callback
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    cassie_vis_t* v = glfwGetWindowUserPointer(window);
    if (action == GLFW_RELEASE) {
        return;
    } else if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_P && mods == 0) {
            printf("attaching camera to pelvis\n");
            v->cam.type = mjCAMERA_TRACKING;
            v->cam.trackbodyid = 1;
            v->cam.fixedcamid = -1;
            mjv_moveCamera(v->m, mjMOUSE_ZOOM, 0.0, -0.05 * 8, &v->scn, &v->cam);
            mjv_moveCamera(v->m, action, 0, -.15, &v->scn, &v->cam);
        }
        // control keys
        if (mods == GLFW_MOD_CONTROL) {
            if (key == GLFW_KEY_A) {
                memcpy(v->cam.lookat, v->m->stat.center, sizeof(v->cam.lookat));
                v->cam.distance = 1.5*v->m->stat.extent;
                // set to free camera
                v->cam.type = mjCAMERA_FREE;
            } else if (key == GLFW_KEY_P) {
                mju_printMat(v->d->qpos, v->m->nq, 1);
            } else if (key == GLFW_KEY_Q) {
                glfwSetWindowShouldClose(window, true);
            }
        }
        // toggle visualiztion flag
        for (int i=0; i < mjNVISFLAG; i++) {
            if (key == mjVISSTRING[i][2][0]) {
                mjtByte flags[mjNVISFLAG];
                memcpy(flags, v->opt.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(v->opt.flags, flags, sizeof(v->opt.flags));
                return;
            }
        }
        // toggle rendering flag
        for (int i=0; i < mjNRNDFLAG; i++) {
            if (key == mjRNDSTRING[i][2][0]) {
                mjtByte flags[mjNRNDFLAG];
                memcpy(flags, v->scn.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(v->scn.flags, flags, sizeof(v->scn.flags));
                return;
            }
        }
        // toggle geom/site group
        for (int i=0; i < mjNGROUP; i++) {
            if (key == i + 48) {    // Int('0') = 48
                if (mods && GLFW_MOD_SHIFT == true) {
                    mjtByte sitegroup[mjNGROUP];
                    memcpy(sitegroup, v->opt.sitegroup, sizeof(sitegroup));
                    sitegroup[i] = sitegroup[i] > 0 ? 0 : 1;
                    // memcpy(v->opt.sitegroup = sitegroup
                    v->opt.sitegroup[i] = sitegroup[i];
                    return;
                } else {
                    mjtByte geomgroup[mjNGROUP];
                    memcpy(geomgroup, v->opt.geomgroup, sizeof(geomgroup));
                    geomgroup[i] = geomgroup[i] > 0 ? 0 : 1;
                    memcpy(v->opt.geomgroup, geomgroup, sizeof(v->opt.geomgroup));
                    return;
                }
            }
        }
        switch (key) {
            case GLFW_KEY_F1: {     // help
                v->showhelp += 1;
                if (v->showhelp > 2) {
                    v->showhelp = 0;
                }
            } break;
            case GLFW_KEY_F2: {     // option
                v->showoption = !v->showoption;
            } break;
            case GLFW_KEY_F3: {     // info
                v->showinfo = !v->showinfo;
            } break;
            case GLFW_KEY_F4: {     // depth
                v->showdepth = !v->showdepth;
            } break;
            case GLFW_KEY_F5: {     // toggle fullscreen
                v->showfullscreen = !v->showfullscreen;
                v->showfullscreen ? glfwMaximizeWindow(window) : glfwRestoreWindow(window);
            } break;
            case GLFW_KEY_F7: {     // sensor figure
                v->showsensor = !v->showsensor;
            } break;
            case GLFW_KEY_ENTER: {  // slow motion
                v->slowmotion = !v->slowmotion;
                v->slowmotion ? printf("Slow Motion Mode!\n") : printf("Normal Speed Mode!\n");
            } break;
            case GLFW_KEY_SPACE: {  // pause
                v->paused = !v->paused;
                v->paused ? printf("Paused\n") : printf("Running\n");
            } break;
            case GLFW_KEY_BACKSPACE: {  // reset
                double qpos_init[35] =
                    {0, 0, 1.01, 1, 0, 0, 0, 0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
                    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
                    -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
                    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};
                double qvel_zero[32] = {0};
                mju_copy(&v->d->qpos[0], qpos_init, 35);
                mju_copy(v->d->qvel, qvel_zero, v->m->nv);
                v->d->time = 0.0;
                mj_forward(v->m, v->d);
            } break;
            case GLFW_KEY_RIGHT: {      // step forward
                if (v->paused) {
                    mj_step(v->m, v->d);
                }
            } break;
            case GLFW_KEY_LEFT: {       // step backw
                if (v->paused) {
                    double dt = v->m->opt.timestep;
                    v->m->opt.timestep = -dt;
                    mj_step(v->m, v->d);
                    v->m->opt.timestep = dt;
                }
            } break;
            case GLFW_KEY_DOWN: {      // step forward 100
                if (v->paused) {
                    for (int i = 0; i < 100; i++) {
                        mj_step(v->m, v->d);
                    }
                }
            } break;
            case GLFW_KEY_UP: {       // step back 100
                if (v->paused) {
                    double dt = v->m->opt.timestep;
                    v->m->opt.timestep = -dt;
                    for (int i = 0; i < 100; i++) {
                        mj_step(v->m, v->d);
                    }
                    v->m->opt.timestep = dt;
                }
            } break;
            case GLFW_KEY_ESCAPE: {     // free camera
                v->cam.type = mjCAMERA_FREE;
            } break;
            case GLFW_KEY_EQUAL: {      // bigger font
                if (fontscale < 200) {
                    fontscale += 50;
                    mjr_makeContext(v->m, &v->con, fontscale);
                }
            } break;
            case GLFW_KEY_MINUS: {      // smaller font
                if (fontscale > 100) {
                    fontscale -= 50;
                    mjr_makeContext(v->m, &v->con, fontscale);
                }
            } break;
            case GLFW_KEY_LEFT_BRACKET: {  // '[' previous fixed camera or free
                int fixedcam = v->cam.type;
                if (v->m->ncam > 0 && fixedcam == mjCAMERA_FIXED) {
                    int fixedcamid = v->cam.fixedcamid;
                    if (fixedcamid  > 0) {
                        v->cam.fixedcamid = fixedcamid-1;
                    } else {
                        v->cam.type = mjCAMERA_FREE;
                    }
                }
            } break;
            case GLFW_KEY_RIGHT_BRACKET: {  // ']' next fixed camera
                if (v->m->ncam > 0) {
                    int fixedcam = v->cam.type;
                    int fixedcamid = v->cam.fixedcamid;
                    if (fixedcam != mjCAMERA_FIXED) {
                        v->cam.type = mjCAMERA_FIXED;
                    } else if (fixedcamid < v->m->ncam - 1) {
                        v->cam.fixedcamid = fixedcamid+1;
                    }
                }
            } break;
        }
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    cassie_vis_t* v = glfwGetWindowUserPointer(window);
    // update button state
    v->button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    v->button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    v->button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);

    // Alt: swap left and right
    if (mods == GLFW_MOD_ALT) {
        bool tmp = v->button_left;
        v->button_left = v->button_right;
        v->button_right = tmp;

        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            button = GLFW_MOUSE_BUTTON_RIGHT;
        } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            button = GLFW_MOUSE_BUTTON_LEFT;
        }
    }

    // update mouse position
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    v->lastx = x;
    v->lasty = y;

    // set perturbation
    int newperturb = 0;
    if (act == GLFW_PRESS && mods == GLFW_MOD_CONTROL && v->pert.select > 0) {
        // right: translate;  left: rotate
        if (v->button_right) {
            newperturb = mjPERT_TRANSLATE;
        } else if (v->button_left) {
            newperturb = mjPERT_ROTATE;
        }
        // perturbation onset: reset reference
        if (newperturb > 0 && v->pert.active == 0) {
            mjv_initPerturb(v->m, v->d, &v->scn, &v->pert);
        }
    }
    v->pert.active = newperturb;

    // detect double-click (250 msec)
    time_t curr_time = time(0);
    if (act == GLFW_PRESS && (curr_time - v->lastclicktm < 0.25) && (button == v->lastbutton)) {
        // determine selection mode
        int selmode = 2;    // Right Click
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            selmode = 1;
        } else if (mods == GLFW_MOD_CONTROL) {
            selmode = 3; // CTRL + Right Click
        }
        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];

        int selgeom = 0;
        int selskin = 0;
        mjtNum aspectratio = (mjtNum) width / height;
        mjtNum relx = (mjtNum) x / width;
        mjtNum rely = (mjtNum) (height - y) / height;

        int selbody = mjv_select(v->m, v->d, &v->opt,
                            aspectratio, relx,
                            rely, 
                            &v->scn, selpnt, &selgeom, &selskin);
        // set lookat point, start tracking is requested
        if (selmode == 2 || selmode == 3) {
            // copy selpnt if geom clicked
            if (selbody >= 0) {
                memcpy(v->cam.lookat, selpnt, sizeof(v->cam.lookat));
            }

            // switch to tracking camera
            if (selmode == 3 && selbody >= 0) {
                v->cam.type = mjCAMERA_TRACKING;
                v->cam.trackbodyid = selbody;
                v->cam.fixedcamid = -1;
            }
        } else { // set body selection
            if (selbody >= 0) {
                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, v->d->qpos+3*selbody);
                mju_mulMatTVec(v->pert.localpos, v->d->xmat+9*selbody, tmp, 3, 3);

                // record selection
                v->pert.select = selbody;
                v->pert.skinselect = selskin;
            } else {
                v->pert.select = 0;
                v->pert.skinselect = -1;
            }
        }

        // stop perturbation on select
        v->pert.active = 0;
    }
    // save info
    if (act == GLFW_PRESS) {
        v->lastbutton = button;
        v->lastclicktm = time(0);
    }
}


// mouse move callback
void mouse_move(GLFWwindow* w, double xpos, double ypos)
{
    cassie_vis_t* v = glfwGetWindowUserPointer(w);

    // no buttons down: nothing to do
    if (!v->button_left && !v->button_middle && !v->button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - v->lastx;
    double dy = ypos - v->lasty;
    v->lastx = xpos;
    v->lasty = ypos;

    int width;
    int height;
    glfwGetWindowSize(w, &width, &height);

    int mod_shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) || glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT);

    // determine action based on mouse button
    int action = mjMOUSE_ZOOM;
    if (v->button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (v->button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }

    // move perturb or camera
    mjtNum xchange = dx / height;
    mjtNum ychange = dy / height;
    if (v->pert.active != 0) {
        mjv_movePerturb(v->m, v->d, action, xchange, ychange, &v->scn, &v->pert);
    } else {
        mjv_moveCamera(v->m, action, xchange, ychange, &v->scn, &v->cam);
    }
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    cassie_vis_t* v = glfwGetWindowUserPointer(window);

    // scroll: emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(v->m, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &v->scn, &v->cam);
}

void window_close_callback(GLFWwindow *window)
{
    cassie_vis_t* v = glfwGetWindowUserPointer(window);
    if (!glfw_initialized || !v || !v->window)
        return;

    // Free mujoco objects
    mjv_freeScene(&v->scn);
    mjr_freeContext(&v->con);

    // Close window
    glfwDestroyWindow(v->window);
    v->window = NULL;
}

void update_scene(cassie_vis_t* v) {
    // clear old perturbations, apply new
    mju_zero(v->d->xfrc_applied, 6 * v->m->nbody);
    if (v->pert.select > 0) {
        mjv_applyPerturbPose(v->m, v->d, &v->pert, 0); // move mocap bodies only
        mjv_applyPerturbForce(v->m, v->d, &v->pert);
    }
    mj_forward(v->m, v->d);

    // update scene and render
    mjv_updateScene(v->m, v->d, &v->opt, &v->pert, &v->cam, mjCAT_ALL, &v->scn);
}

void render(cassie_vis_t* v) {
    
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(v->window, &viewport.width, &viewport.height);
    mjr_render(viewport, &v->scn, &v->con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(v->window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

cassie_vis_t* cassie_vis_init(mjModel* m, mjData* d) {
    // Allocate visualization structure
    cassie_vis_t *v = malloc(sizeof (cassie_vis_t));

    // Set interaction ctrl vars
    v->lastx = 0.0;
    v->lasty = 0.0;
    v->button_left = false;
    v->button_middle = false;
    v->button_right = false;
    v->lastbutton = GLFW_MOUSE_BUTTON_1;
    v->lastclicktm = 0.0;
    // GLFWvidmode* vidmode = glfwGetVideoMode_fp(glfwGetPrimaryMonitor_fp());
    v->refreshrate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;
    v->showhelp = 0;
    v->showoption = false;
    v->showdepth = false;
    v->showfullscreen = false;
    v->showsensor = false;
    v->slowmotion = false;
    v->showinfo = true;
    v->paused = true;
    v->framenum = 0;
    v->lastframenum = 0;
    v->m = m;
    v->d = d;

    // create window, make OpenGL context current, request v-sync
    v->window = glfwCreateWindow(1200, 900, "Cassie ZMP", NULL, NULL);
    glfwMakeContextCurrent(v->window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&v->cam);
    mjv_defaultOption(&v->opt);
    mjr_defaultContext(&v->con);
    mjv_defaultScene(&v->scn);

    // create scene and context
    mjv_makeScene(m, &v->scn, 2000);
    mjr_makeContext(m, &v->con, fontscale);

    // Set callback for user-initiated window close events
    glfwSetWindowUserPointer(v->window, v);
    glfwSetWindowCloseCallback(v->window, window_close_callback);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(v->window, key_callback);
    glfwSetCursorPosCallback(v->window, mouse_move);
    glfwSetMouseButtonCallback(v->window, mouse_button);
    glfwSetScrollCallback(v->window, scroll);

    return v;
}