#ifndef LINEAR_MODEL_H
#define LINEAR_MODEL_H

#include <string>

#include "model_interface/ModelInterface.h"
#include "model_interface/ModelData.h"
/**
 * Class used to load and query FVCOM data
 */
class LinearModel : public ModelInterface
{
public:

    /**
     * Initalize ConstantModel class with zeros
     */
    LinearModel();

    /**
     * Initalize ConstantModel class with provided values
     */
    LinearModel(float u, float v, float temp, float salt, float dye, float depth,
                  float zeroDistance, float centerX, float centerY, std::string type);



    const ModelData getData(float x, float y, float height, float time);
    const ModelData getDataOutOfRange(float x, float y, float height, float time);

private:
    
    float u;
    float v;
    float temp;
    float salt;
    float dye;
    float depth;
    float zeroDistance;
    float centerX;
    float centerY;
    std::string type;

};

#endif
