#ifndef CONSTANT_MODEL_H
#define CONSTANT_MODEL_H

#include "model_server/ModelInterface.h"
#include "model_server/ModelData.h"
/**
 * Class used to load and query FVCOM data
 */
class ConstantModel : public ModelInterface
{
public:

	/**
	 * Initalize ConstantModel class with zeros
     */
	ConstantModel();

	/**
	 * Initalize ConstantModel class with provided values
     */
	ConstantModel(float u, float v, float temp, float salt, float dye);



	const ModelData getData(float x, float y, float height, float time);

private:
	
	float u;
	float v;
	float temp;
	float salt;
	float dye;

};

#endif
