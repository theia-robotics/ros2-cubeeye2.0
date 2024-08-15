#ifndef ICUBE_MODEL_PARAMETER_H_
#define ICUBE_MODEL_PARAMETER_H_

#include "ModelParameter.h"

class ICubeModelParameter : public ModelParameter
{
public:
    ICubeModelParameter(meere::sensor::sptr_camera camera);
};

#endif // ICUBE_MODEL_PARAMETER_H_
