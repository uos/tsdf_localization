/**
 * @file naiv_evaluation.h
 * @author Marc Eiosldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Class that implements a naiv evaluation model
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef NAIV_EVALUATION_H
#define NAIV_EVALUATION_H

#include <tsdf_localization/evaluation/model/evaluation_model.h>

namespace tsdf_localization
{

/**
 * @brief Class that implements a naiv evaluation model
 * 
 */
class NaivEvaluation : public EvaluationModel
{
    // Calculated result of the naiv evaluation model by condering all inserted measurements
    FLOAT_T m_error;

  public:
    /**
     * @brief Construct a new Naiv Evaluation object
     * 
     */
    NaivEvaluation();

    /**
     * @brief Insert a range measurement of the environment to the evaluation model by comparing with a simulated range 
     * 
     * @param observed_range Measured range in the real environment
     * @param simulated_range Range simulated in the used map
     */
    virtual void insertMeasurement(FLOAT_T observed_range, FLOAT_T simulated_range);

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
    virtual FLOAT_T evaluate();

    /**
     * @brief Resets the current evaluation procedure of scan
     * 
     */
    virtual void reset();
};

} // namespace tsdf_localization

#endif