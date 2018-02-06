#include "constant_model/ConstantModel.h"
#include "model_server/ModelData.h"
#include "ros/ros.h"

#include <stdexcept>
#include <math.h>

ConstantModel::ConstantModel() :
 	u(0),
 	v(0),
 	temp(0),
 	salt(0),
 	dye(0)
{}

ConstantModel::ConstantModel(float u, float v, float temp, float salt, float dye, float depth) :
 	u(u),
 	v(v),
 	temp(temp),
 	salt(salt),
 	dye(dye),
    depth(depth)
{}

const ModelData ConstantModel::getData(float x, float y, float height, float time)
{
	ModelData data;
	
	data.u = u;
	data.v = v;
	data.temp = temp;
	data.salt = salt;
	data.dye = dye;
    data.depth = depth;

	return data;
}

