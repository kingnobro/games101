//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
// the direction of ray emits from eye_pos to intersection.coords
// lightSampled: contribution from other reflectors should not sample from lights again
Vector3f Scene::shade(Intersection& intersection, Vector3f wo) const
{
    Material *m = intersection.m;
    // hit the light
    if (m->hasEmission()) return m->getEmission();

    Vector3f N = intersection.normal;
    Vector3f p = intersection.coords;
    
    // Contribution from the light source
    float pdf_light;
    Intersection lightPos;
    Scene::sampleLight(lightPos, pdf_light);

    Vector3f x = lightPos.coords;   // hit position on the Light
    Vector3f NN = lightPos.normal;  // normal of the Light
    Vector3f emit = lightPos.emit;  // m_emission of Light
    Vector3f light2obj = p - x;
    Vector3f wi = normalize(light2obj);
    float dis = light2obj.norm();

    // shoot a ray from p to x
    Intersection block = Scene::intersect(Ray(p, -wi));
    Vector3f L_dir = Vector3f();
    // if the ray is not blocked in the middle
    if (std::fabs(block.distance - dis) < EPSILON) {
        // note: cosine value should greater than 0
        float cos_theta = std::max(0.0f, dotProduct(-wi, N));
        float cos_theta_x = std::max(0.0f, dotProduct(wi, NN));
        Vector3f f_r = m->eval(wi, wo, N);
        L_dir = emit * f_r * cos_theta * cos_theta_x / (dis * dis) / pdf_light;
    }
    
    // Contribution from other relectors
    // Test Russian Roulette with probability RussianRoulette
    if (get_random_float() > RussianRoulette) return L_dir;
    Vector3f L_indir = Vector3f();
    wi = m->sample(wo, N).normalized();
    block = Scene::intersect(Ray(p, wi));
    if (block.happened && !block.obj->hasEmit()) {
        float pdf = m->pdf(wi, wo, N);
        Vector3f f_r = m->eval(wi, wo, N);
        float cos = std::max(0.0f, dotProduct(wi, N));
        L_indir = shade(block, -wi) * f_r * cos / pdf / RussianRoulette;
    }
    
    return L_dir + L_indir;
}

Vector3f Scene::castRay(const Ray &ray) const
{
    Intersection intersection = Scene::intersect(ray);
    if (!intersection.happened) return Vector3f();
    return Scene::shade(intersection, -ray.direction);
}