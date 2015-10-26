#ifndef SPHERE_H
#define SPHERE_H

#include "Vector3.h"
#include "Object.h"

class Sphere : public Object
{
    public:
        Sphere();
        virtual ~Sphere();

        void setCenter(const Vector3& v)    {m_center = v;}
        void setRadius(const float f)       {m_radius = f;}

        const Vector3& center() const       {return m_center;}
        float radius() const                {return m_radius;}

        virtual void renderGL();
        virtual bool intersect(HitInfo& result, const Ray& ray,
                               float tMin = 0.0f, float tMax = MIRO_TMAX, bool shadow = false) override;
    
        Vector3 getCentroid() override;
        Vector3 getMin() override;
        Vector3 getMax() override;

    protected:
        Vector3 m_center;
        float m_radius;
};

#endif // SPHERE_H
