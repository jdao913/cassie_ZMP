/*  Copyright Â© 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/


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
#include <math.h>

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data

int pelvis_body_id = 1;
int floor_geom_id = 0;
int left_geom_id = 18;
int right_geom_id = 34;
mjContact left_contact[2];
mjContact right_contact[2];
double left_front[3];
double left_back[3];
double right_front[3];
double right_back[3];
double foot_length = 0.051423;

void set_ZMP(mjData* d) {
    double cassie_com[3];
    // Center of mass is given in (y, z, x)
    for (int i = 0; i < 3; i++) {
        cassie_com[i] = d->subtree_com[pelvis_body_id+i];
    }
    d->qpos[35] = cassie_com[2];
    d->qpos[36] = cassie_com[0];
    d->qpos[37] = 0;//cassie_com[1];
}

// Geom 36 is front_foot_line, geom 37 is back_foot_line
void set_support(cassie_vis_t* v) {
    mjModel* m = v->m;
    mjData* d = v->d;
    // Check all contacts for contact between foot and ground
    int num_left = 0;
    int num_right = 0;
    for (int i = 0; i < d->ncon; i++) {
        mjContact curr_con = d->contact[i];
        int geom1 = curr_con.geom1;
        int geom2 = curr_con.geom2;
        if ((geom1 == floor_geom_id && geom2 == left_geom_id) || (geom1 == left_geom_id && geom2 == floor_geom_id)) {
            left_contact[num_left] = curr_con;
            num_left++;
        } else if ((geom1 == floor_geom_id && geom2 == right_geom_id) || (geom1 == right_geom_id && geom2 == floor_geom_id)) {
            right_contact[num_right] = curr_con;
            num_right++;
        }
    }        
    if (num_left == 0 || num_right == 0) {
        // No foot contacts, make geoms invisible
        printf("Not that both feet are in contact, making geoms invisible\n");
        m->geom_rgba[36*4+3] = 0;
        m->geom_rgba[37*4+3] = 0;
    } else if ((num_left == 1 && num_right == 1)) {
        printf("Only one contact each, making one geom invisible");
        m->geom_rgba[36*4+3] = 0;
        mjvGeom* g = v->scn.geoms + v->scn.ngeom++;
        float rgba[4] = {1, .8, .2, .7};
        for (int i = 0; i < 4; i++) {
            g->rgba[i] = rgba[i];
        }
        mjtNum* left_pos = left_contact[0].pos;
        mjtNum* right_pos = right_contact[0].pos;
        mjv_makeConnector(g, mjGEOM_CAPSULE, .01, left_pos[0], left_pos[1], left_pos[2], right_pos[0], right_pos[1], right_pos[2]);
    } else if (num_left == 2 && num_right == 1) {
        printf("left triangle contact\n");
        mjtNum* left_pos1 = left_contact[0].pos;
        mjtNum* left_pos2 = left_contact[1].pos;
        mjtNum* right_pos = right_contact[0].pos;
        mjvGeom* g = v->scn.geoms + v->scn.ngeom++;
        float rgba[4] = {1, .8, .2, .7};
        for (int i = 0; i < 4; i++) {
            g->rgba[i] = rgba[i];
        }
        mjv_makeConnector(g, mjGEOM_CAPSULE, .01, left_pos1[0], left_pos1[1], left_pos1[2], right_pos[0], right_pos[1], right_pos[2]);
        mjvGeom* g2 = v->scn.geoms + v->scn.ngeom++;
        for (int i = 0; i < 4; i++) {
            g2->rgba[i] = rgba[i];
        }
        mjv_makeConnector(g2, mjGEOM_CAPSULE, .01, left_pos2[0], left_pos2[1], left_pos2[2], right_pos[0], right_pos[1], right_pos[2]);
    } else if (num_left == 1 && num_right == 2) {
        printf("right triangle contact\n");
        mjtNum* left_pos1 = left_contact[0].pos;
        mjtNum* right_pos1 = right_contact[0].pos;
        mjtNum* right_pos2 = right_contact[1].pos;
        mjvGeom* g = v->scn.geoms + v->scn.ngeom++;
        float rgba[4] = {1, .8, .2, .7};
        for (int i = 0; i < 4; i++) {
            g->rgba[i] = rgba[i];
        }
        mjv_makeConnector(g, mjGEOM_CAPSULE, .01, left_pos1[0], left_pos1[1], left_pos1[2], right_pos1[0], right_pos1[1], right_pos1[2]);
        mjvGeom* g2 = v->scn.geoms + v->scn.ngeom++;
        for (int i = 0; i < 4; i++) {
            g2->rgba[i] = rgba[i];
        }
        mjv_makeConnector(g2, mjGEOM_CAPSULE, .01, left_pos1[0], left_pos1[1], left_pos1[2], right_pos2[0], right_pos2[1], right_pos2[2]);
    } else if (num_left == 2 && num_right == 2) {
        printf("rectangle contact\n");
        mjtNum* left_pos1 = left_contact[0].pos;
        mjtNum* left_pos2 = left_contact[1].pos;
        mjtNum* right_pos1 = right_contact[0].pos;
        mjtNum* right_pos2 = right_contact[1].pos;
        mjvGeom* g = v->scn.geoms + v->scn.ngeom++;
        float rgba[4] = {1, .8, .2, .7};
        for (int i = 0; i < 4; i++) {
            g->rgba[i] = rgba[i];
        }
        mjv_makeConnector(g, mjGEOM_CAPSULE, .01, left_pos1[0], left_pos1[1], left_pos1[2], right_pos1[0], right_pos1[1], right_pos1[2]);
        mjvGeom* g2 = v->scn.geoms + v->scn.ngeom++;
        for (int i = 0; i < 4; i++) {
            g2->rgba[i] = rgba[i];
        }
        mjv_makeConnector(g2, mjGEOM_CAPSULE, .01, left_pos2[0], left_pos2[1], left_pos2[2], right_pos2[0], right_pos2[1], right_pos2[2]);
    }
    // mjvGeom* g = v->scn.geoms + v->scn.ngeom++;
    // float rgba[4] = {1, .8, .2, .7};
    // for (int i = 0; i < 4; i++) {
    //     g->rgba[i] = rgba[i];
    // }
    // mjv_makeConnector(g, mjGEOM_CAPSULE, .01, 0, 0, 0, 1, .3, 0);
}

void pelvis_hold(mjModel* m, mjData* d)
{
    // Set stiffness/damping for body translation joints
    for (int i = 0; i < 3; ++i) {
        m->jnt_stiffness[i] = 1e5;
        m->dof_damping[i] = 1e4;
        m->qpos_spring[i] = d->qpos[i];
    }

    // Set damping for body rotation joint
    for (int i = 3; i < 7; ++i)
        m->dof_damping[i] = 1e3;
}

// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // activate software
    const char* key_buf = getenv("MUJOCO_KEY_PATH");
    mj_activate(key_buf);

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);
    printf("made mjdata\n");
    printf("num of geoms: %i\n", m->ngeom);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
    glfw_initialized = true;

    cassie_vis_t* v = cassie_vis_init(m, d);

    // Setup additional MuJoCo stuff for ZMP calcs
    pelvis_body_id = mj_name2id(v->m, mjOBJ_BODY, "cassie-pelvis");
    // printf("floor id: %i\n", mj_name2id(v->m, mjOBJ_BODY, "floor"));

    // mj_step(m, d);
    double qpos_init[35] =
                    {0, 0, 1.01, 1, 0, 0, 0, 0.0045, 0, 0.4973, 0.9785, -0.0164, 0.01787, -0.2049,
                    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968,
                    -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
                    -1.1997, 0, 1.4267, 0, -1.5244, 1.5244, -1.5968};
    double qvel_zero[32] = {0};
    mju_copy(&v->d->qpos[0], qpos_init, 35);
    mju_copy(v->d->qvel, qvel_zero, v->m->nv);
    v->d->time = 0.0;
    // v->d->qpos[36] = 1;
    set_ZMP(v->d);
    mj_forward(v->m, v->d);
    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(v->window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        if (!v->paused) {
            while( d->time - simstart < 1.0/60.0 ) {
                mj_step(v->m, v->d);
                // v->d->qpos[37] = 1;
                
                // v->m->geom_rgba[37*4] = 1;
            }
            set_ZMP(v->d);
            // set_support(v);
        }
        update_scene(v);
        set_support(v);
        render(v);
        
    }

    //free visualization storage
    mjv_freeScene(&v->scn);
    mjr_freeContext(&v->con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}