/**
 * @file likelihood_evaluation.h
 * @author Marc Eiosldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Class that implements a likelihood model
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef LIKELIHOODEVALUATION_H
#define LIKELIHOODEVALUATION_H

#include <evaluation/model/evaluation_model.h>

namespace mcl
{

constexpr float SIGMA = 0.1f;
constexpr float SIGMA_QUAD = SIGMA * SIGMA;

/**
* @brief Class that implements a likelihood model
* 
*/
class LikelihoodEvaluation : public EvaluationModel
{
  // Sum of scan point evaluations
  FLOAT_T m_sum;
  // Maximum range of a considered scan point
  FLOAT_T m_range_max;

  public:
    /**
     * @brief Construct a new Likelihood Evaluation object
     * 
     * @param range_max Maximum range of an inserted measurement. All measurements with a higher range are ignored
     */
    LikelihoodEvaluation(FLOAT_T range_max);

    /**
     * @brief Insert a range measurement of the environment to the evaluation model by comparing with a simulated range 
     * 
     * @param observed_range Measured range in the real environment
     * @param simulated_range Range simulated in the used map
     */
    virtual void insertMeasurement(FLOAT_T observed_range, FLOAT_T simulated_range) override;

    /**
    * @brief Insert a range measurement of the environment to the evaluation model
    * 
    * @param measurement Measured range in the real environment
    */
    virtual void insertMeasurement(FLOAT_T measurement) override;
    
    /**
    * @brief Get the evaluation result consindering all inserted measurements after the last reset
    * 
    * @return FLOAT_T Result of the evaluation model
    */
    virtual FLOAT_T evaluate() override;

    /**
    * @brief Resets the current evaluation procedure of scan
    * 
    */
    virtual void reset() override;

};

} // namespace mcl

#endif