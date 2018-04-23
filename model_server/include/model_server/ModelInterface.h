#ifndef MODEL_INTERFACE_H
#define MODEL_INTERFACE_H

#include "model_server/ModelData.h"

class ModelInterface
{
public:

	ModelInterface() {}
	virtual ~ModelInterface() {}

	virtual const ModelData getData(float x, float y, float height, float time)=0;
    virtual const ModelData getDataOutOfRange(float x, float y, float height, float time)=0;
};

#endif