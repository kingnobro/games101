#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D step;
        if (num_nodes > 1) step = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; i++)
        {
            Vector2D pos = start + step * i;
            Mass* mass = new Mass(pos, node_mass, false);
            masses.push_back(mass);
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
        for (int i = 0; i < num_nodes - 1; i++)
        {
            Spring* s = new Spring(masses[i], masses[i+1], k);
            springs.push_back(s);
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Mass *a = s->m1;
            Mass *b = s->m2;
            Vector2D diff = b->position - a->position;
            Vector2D force = -s->k * diff * (diff.norm() - s->rest_length) / diff.norm();
            b->forces += force;
            a->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * delta_t;
                auto a = m->forces / m->mass;
                m->velocity += a * delta_t;
                // semi-implicit method
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Mass *a = s->m1;
            Mass *b = s->m2;
            Vector2D diff = b->position - a->position;
            Vector2D force = -s->k * diff * (diff.norm() - s->rest_length) / diff.norm();
            b->forces += force;
            a->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * delta_t;
                Vector2D temp_position = m->position;
                float damping_factor = 0.00005;
                auto a = m->forces / m->mass;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                m->position = temp_position + (1 - damping_factor) * (temp_position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
