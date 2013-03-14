#include "boid.h"
#include "Vector.h"

Boid::Boid()
{
    this->max_speed=20;
    this->min_speed=1;
    this->mass=10;
    this->position.x=(rand()%3000)-1500;
    this->position.y=(rand()%3000)-1500;
    this->position.z=(rand()%3000)-1500;
    this->direction.x=((rand()%200)/200.0)-1;
    this->direction.y=((rand()%200)/200.0)-1;
    this->direction.z=((rand()%200)/200.0)-1;
    this->speed=10;
    this->scale=30.0;
    this->cubeHalfSize=1500;

}

void Boid::update_position(Vector steering_force)
{
    Vector acceleration = steering_force/this->mass;
    this->velocity+=acceleration;
    this->position+=this->velocity;
}

void Boid::update_local()
{
    this->direction.Normalize();
    double m11=this->direction.x;
    double m21=this->direction.y;
    double m31=this->direction.z;

    Vector tempUp=Vector(0,1,0);
    Vector newZ=this->direction.Cross(tempUp);
    newZ.Normalize();
    double m13=newZ.x;
    double m23=newZ.y;
    double m33=newZ.z;

    Vector newY=newZ.Cross(this->direction);
    newY.Normalize();
    double m12=newY.x;
    double m22=newY.y;
    double m32=newY.z;

    this->rotAngle=(((360/(2*3.14159265)))*(acos(0.5*(m11+m22+m33-1))));
    double denom=2*(sin((rotAngle*3.14159265)/180));

    this->ux=(m32-m23)/denom;
    this->uy=(m13-m31)/denom;
    this->uz=(m21-m12)/denom;

    Vector rotAxis=Vector(ux,uy,uz);
    rotAxis.Normalize();
}

void Boid::update_separation()
{
    Vector temp_dist = this->avrg_neigh_pos - this->position;
    Vector dist = (this->position - this->neigh_1_pos) + (this->position - this->neigh_2_pos) + (this->position - this->neigh_3_pos) + (this->position - this->neigh_4_pos);
    dist.x /= 4;
    dist.y /= 4;
    dist.z /= 4;
    if((temp_dist.Length())<250)
    {
        this->unit_avoid.x=dist.x/(625*2);
        this->unit_avoid.y=dist.y/(625*2);
        this->unit_avoid.z=dist.z/(625*2);
        this->unit_avoid.Normalize();
    }
    else
    {
        this->unit_avoid.x=0;
        this->unit_avoid.y=0;
        this->unit_avoid.z=0;
    }
}

void Boid::update_cohesion()
{
    Vector temp_vel = this->avrg_neigh_pos - this->position;
    if((temp_vel.Length())>150)
    {
        this->unit_centre.x = temp_vel.x / (625*2);
        this->unit_centre.y = temp_vel.y / (625*2);
        this->unit_centre.z = temp_vel.z / (625*2);
        this->unit_centre.Normalize();
    }
    else
    {
        this->unit_centre.x = 0;
        this->unit_centre.y = 0;
        this->unit_centre.z = 0;
    }
}

void Boid::update_alignment()
{
    Vector temp_vel = this->avrg_neigh_vel + this->direction;
    temp_vel.x /= 2;
    temp_vel.y /= 2;
    temp_vel.z /= 2;
    double temp_ang = this->avrg_neigh_vel.AngleBetween(this->direction);
    if(temp_ang>0.25)
    {
        this->avrg_neigh_vel.x = temp_vel.x / (625*2);
        this->avrg_neigh_vel.y = temp_vel.y / (625*2);
        this->avrg_neigh_vel.z = temp_vel.z / (625*2);
        this->avrg_neigh_vel.Normalize();
    }
    else
    {
        this->avrg_neigh_vel.x = 0;
        this->avrg_neigh_vel.y = 0;
        this->avrg_neigh_vel.z = 0;
    }
}

