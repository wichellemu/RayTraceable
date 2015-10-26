#ifndef BVH_H
#define BVH_H

#include "Miro.h"
#include "Object.h"
#include "BVHNode.h"

class BVH
{
    public:
        ~BVH();

        void build(Objects * objs);

        bool intersect(HitInfo& result, const Ray& ray,
        float tMin = 0.0f, float tMax = MIRO_TMAX, bool shadow = false) const;
        
        BVHNode * root;
        //static int rayTotal;
        //static void resetStats() { rayTotal = 0; }

    protected:
        Objects * m_objects;
};

#endif // BVH_H
