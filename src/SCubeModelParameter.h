#ifndef SCUBE_MODEL_PARAMETER_H_
#define SCUBE_MODEL_PARAMETER_H_

#include "ModelParameter.h"

class SCubeModelParameter : public ModelParameter
{
public:
    SCubeModelParameter(meere::sensor::sptr_camera camera);
};

#endif // SCUBE_MODEL_PARAMETER_H_
