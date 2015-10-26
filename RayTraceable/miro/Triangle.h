#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "Object.h"
#include "Lambert.h"

/*
   The Triangle class stores a pointer to a mesh and an index into its
   triangle array. The mesh stores all data needed by this Triangle.
   */
class Triangle : public Object
{
    public:
        Triangle(TriangleMesh * m = 0, unsigned int i = 0);
        virtual ~Triangle();

        void setIndex(unsigned int i) {m_index = i;}
        void setMesh(TriangleMesh* m) {m_mesh = m;}

        virtual void renderGL();
        virtual bool intersect(HitInfo& result, const Ray& ray,
            float tMin = 0.0f, float tMax = MIRO_TMAX, bool shadow = false) override;
        
        Vector3 getCentroid() override;
        Vector3 getMin() override;
        Vector3 getMax() override;
        Vector3 sample(Vector3 * normal = nullptr);       // Random sample in triangle, puts normal result in parameter

    protected:
        TriangleMesh* m_mesh;
        unsigned int m_index;

};

#endif // TRIANGLE_H
