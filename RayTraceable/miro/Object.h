#ifndef OBJECT_H
#define OBJECT_H

#include <vector>
#include "Miro.h"
#include "Material.h"
#include "Vector3.h"

class Object
{
    public:
        Object() {}
        virtual ~Object() {}

        void setMaterial(const Material* m) {m_material = m;}

        virtual void renderGL() {}
        virtual void preCalc() {}

        virtual bool intersect(HitInfo& result, const Ray& ray,
            float tMin = 0.0f, float tMax = MIRO_TMAX, bool shadow = false) = 0;

        virtual Vector3 getCentroid() = 0;
        virtual Vector3 getMin() = 0;
        virtual Vector3 getMax() = 0;
        float prevsize;                           

    protected:
        const Material* m_material;
};

typedef std::vector<Object*> Objects;

#endif // OBJECT_H

