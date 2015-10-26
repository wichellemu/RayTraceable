#ifndef CSE168_MATERIAL_H_INCLUDED
#define CSE168_MATERIAL_H_INCLUDED

#include "Miro.h"
#include "Vector3.h"

class Material
{
    public:
        Material();
        virtual ~Material();

        virtual void preCalc() {}
    
        virtual Vector3 shade(const Ray& ray, const HitInfo& hit,
                              const Scene& scene) const;
                          
        void setShadowable(bool s) { m_shadowable = s; }
        bool shadowable() const { return m_shadowable; }

    protected:
        bool m_shadowable = true;
};

#endif // CSE168_MATERIAL_H_INCLUDED
