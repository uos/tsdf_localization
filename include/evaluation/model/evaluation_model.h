/**
 * @file evaluation_model.h
 * @author Marc Eiosldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Interface to represent a class that can implement an evaluation model in the Monte Carlo Localization
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef EVALUATION_MODEL_H
#define EVALUATION_MODEL_H

#include <util/util.h>

namespace mcl
{

/**
 * @class EvaluationModel
 * 
 * @brief Interface for a model for comparing two scans
 */
class EvaluationModel
{
  public:
    /**
     * @brief Is called for every observed scan range with the corresponding expected scan range 
     */
    virtual void insertMeasurement(FLOAT_T observed_range, FLOAT_T simulated_range) = 0;

    /**
     * @brief Is called for every observed scan range with the corresponding expected scan range 
     */
    virtual void insertMeasurement(FLOAT_T measurement) = 0;

    /**
     * @brief Is called after comparing every scan range for getting a probability, 
     *        which represents how good an observed scan matches a simulated scan     
     */ 
    virtual FLOAT_T evaluate() = 0;

    /**
     * @brief Is called before comparing two new scans 
     */
    virtual void reset() = 0;
};

} // namespace mcl

#endif