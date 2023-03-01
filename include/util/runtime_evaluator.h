#pragma once

/**
 * @author Marc Eisoldt
 * @author Steffen Hinderink
 */

#include <vector> // for the vector of measurement variables
#include <memory> // for singleton pointer
#include <exception> // for custom exception
#include <limits> // for maximum value
#include <string>

#include <util/time_util.h>
#include <util/filter.h>

/**
 * Define this token to enable time measurements in the whole project.
 * Every measurement must be enclosed in
 * #ifdef TIME_MEASUREMENT
 * ...
 * #endif
 */
#define TIME_MEASUREMENT

using measurement_unit = std::chrono::microseconds;

namespace mcl
{

/**
 * Helper struct for managing the different measurement variables for every measured task.
 */
struct EvaluationFormular
{
    /**
     * Constructor for the evaluation formular. All measurement variables are initialized.
     * @param name Name of the task
     * @param be_recorded Should the measurements be recorded?
     */
    EvaluationFormular(const std::string& name, bool be_recorded = false) :
        name(name), active(false), accumulate(0),
        count(0), last(0), sum(0), min(std::numeric_limits<unsigned long long>::max()), max(0),
        filter(100), recorded(be_recorded) 
        {

        }
    /// Name of the task that is used as an identifier for the measurement variables
    std::string name;
    /// Flag that indicates whether the task is currently being measured (i.e. it is between start and stop)
    bool active;
    /// Accumulated runtime for the task. The runtime of the methods of the evaluator are not included, so that the evaluator is transparent.
    unsigned long long accumulate;

    /// Number of measurements
    unsigned int count;
    /// Last measured runtime
    unsigned long long last;
    /// Sum of all measured runtimes
    unsigned long long sum;
    /// Minimum of all measured runtimes
    unsigned long long min;
    /// Maximum of all measured runtimes
    unsigned long long max;
    /// Gives an Average of the last 100 measurements
    SlidingWindowFilter<double> filter;
    /// Should the measurements be recorded for over time analysis?
    bool recorded;
    /// All measurements during the evaluation process
    std::vector<unsigned long long> recorded_data;
};

/**
 * Custom exception that is thrown when the protocol of calling start and stop is not followed.
 */
struct RuntimeEvaluationException : public std::exception
{
    /**
     * Returns the message, that gives information over what caused the exception to occur.
     * @return Exception message
     */
    const char* what() const throw()
    {
        return "Runtime evaluation exception:\nStart was called for an already started measurement or stop was called before calling start!";
    }
};

/**
 * Encapsulates the runtime measurement for different tasks.
 * Different tasks can be measured at the same time and nested. They are distinguished by names.
 * The evaluation is transparent in the runtime measurement, i.e. the runtime of the evaluator itself is not measured.
 * This is achieved by measuring the time only in between every call of the functions and
 * updating the measurements of those tasks, whose measurements are currently active, accordingly.
 * The evaluator is implemented as a singleton (except for the public constructor for measurements in a different thread).
 */
class RuntimeEvaluator
{

public:

    /**
     * Returns the singleton instance. Creates a new one the first time.
     * @return Singleton instance
     */
    static RuntimeEvaluator& get_instance();

    /**
     * Default Destructor
     */
    ~RuntimeEvaluator() = default;

    /**
     * Don't allow copies of the instance by copy constructor to ensure singleton property.
     */
    RuntimeEvaluator(RuntimeEvaluator&) = delete;

    /// delete copy constructor
    RuntimeEvaluator(const RuntimeEvaluator&) = delete;

    /// delete move constructor
    RuntimeEvaluator(RuntimeEvaluator&&) = delete;

    /**
     * Don't allow copies of the instance by assignment operator to ensure singleton property.
     */
    RuntimeEvaluator& operator=(RuntimeEvaluator&) = delete;

    /// delete move assignment operator
    RuntimeEvaluator& operator=(RuntimeEvaluator&&) = delete;

    /**
     * @brief Clear all running time measurements
     */
    void clear();


    /**
     * Starts a new measurent for a task.
     * @param task_name Name of the task, which will be shown in the printed overview
     *                  and is used as an identifier to find the right measurement variables
     * @param recorded Should the measurement be recorded for analysis over time?
     * @throws RuntimeEvaluationException if a measurement for that task was already started
     */
    void start(const std::string& task_name, bool recorded = false);

    /**
     * Stops the measurement and updates the variables for a task.
     * This function has to be called after the start function was called for that task.
     * @param task_name Name of the task, which will be shown in the printed overview
     *                  and is used as an identifier to find the right measurement variables
     * @throws RuntimeEvaluationException if the measurement for that task has not been started yet
     */
    void stop(const std::string& task_name);

    /**
     * Getter for the vector of the different measurement variables for every measured task
     * @return Vector of the measurement variables
     */
    const std::vector<EvaluationFormular> get_forms();

    /**
     * Returns a string that contains a table with the measurements for every measured task.
     * @return String that contains a table with all measurements
     */
    std::string to_string(bool with_recorded = false);

    /**
     * Do not use this constructor except in a different thread!
     * This was previously private to ensure singleton property.
     */
    RuntimeEvaluator();

private:

    /**
     * Pauses the time measurements. The time that has past since the last resume is accumulated for every currently measured task.
     * This method is called at the beginning of every public method.
     */
    void pause();

    /**
     * Resumes the time measurements by simply storing the current time so that it can be used at the next pause.
     * This method is called at the end of every public method.
     */
    void resume();

    /**
     * Try to find the formular with the given task name
     *
     * @param task_name Task name of the wanted formular
     * @return int if found, the index of the formular
     *             else -1
     */
    int find_formular(const std::string& task_name);

    /// Vector of the different measurement variables for every measured task
    std::vector<EvaluationFormular> forms_;

    /// Histogram of the total time
    std::vector<int> histogram_;
    const int HIST_BUCKET_SIZE = 10;

    /// Excess time from the previous Scans. Used to calculate how many Scans are dropped
    double overhang_time_;

    /// Number of Scans that were dropped because previous Scans took too long
    int num_dropped_scans;

    /// Number of Scans that exceeded the 50ms limit
    int num_scans_over_50ms;

    /// Temporary variable for storing the start time of the current measurement
    std::chrono::_V2::system_clock::time_point start_;
};

/**
 * Puts a by a given evaluator (using its to_string method) into a given output stream.
 * @param os Output stream in which the evaluator is put
 * @param evaluator Runtime evaluator that is put into the stream
 * @return Output stream with the evaluator
 */
std::ostream& operator<<(std::ostream& os, RuntimeEvaluator& evaluator);

} // namespace mcl