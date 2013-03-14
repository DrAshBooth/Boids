#ifndef BOID_H
#define BOID_H

#include "vector.h"

class Boid
{
private:
    double min_speed;
    double max_speed;
    double mass;
    double speed;
    double scale;
    double cubeHalfSize;

    Vector unit_avoid;
    Vector unit_centre;
    Vector unit_av_vel;
    Vector avrg_neigh_pos;
    Vector avrg_neigh_vel;

public:
    double ux;
    double uy;
    double uz;
    double rotAngle;
    Vector position;
    Vector direction;
    Vector velocity;

    int neigh_1;
    int neigh_2;
    int neigh_3;
    int neigh_4;
    Vector neigh_1_pos;
    Vector neigh_2_pos;
    Vector neigh_3_pos;
    Vector neigh_4_pos;
    Vector neigh_1_dir;
    Vector neigh_2_dir;
    Vector neigh_3_dir;
    Vector neigh_4_dir;

    Boid();

    void update_position(Vector steering_force);
    void update_local();
    void update_separation();
    void update_cohesion();
    void update_alignment();
};

#endif // BOID_H
