/**
 * @file runtime_evaluator.cpp
 * @author Marc Eisoldt
 * @author Steffen Hinderink
 */

#include <util/runtime_evaluator.h>

#include <sstream> // for output
#include <iomanip> // for formatting
#include <math.h>

using namespace std::chrono;

namespace mcl
{

RuntimeEvaluator& RuntimeEvaluator::get_instance()
{
    static RuntimeEvaluator instance;
    return instance;
}

RuntimeEvaluator::RuntimeEvaluator() : forms_(), histogram_(8, 0)
{
    overhang_time_ = 0.0;
    num_dropped_scans = num_scans_over_50ms = 0;
    // "unpause" for the first time
    start_ = high_resolution_clock::now();
}

void RuntimeEvaluator::pause()
{
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<measurement_unit>(stop - start_);

    // add new interval to all active measurements
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].active)
        {
            forms_[i].accumulate += duration.count();
        }
    }
}

void RuntimeEvaluator::resume()
{
    start_ = high_resolution_clock::now();
}

int RuntimeEvaluator::find_formular(const std::string& task_name)
{
    for (uint i = 0; i < forms_.size(); i++)
    {
        if (forms_[i].name == task_name)
        {
            return i;
        }
    }

    return -1;
}

void RuntimeEvaluator::start(const std::string& task_name, bool recorded)
{
#ifdef TIME_MEASUREMENT
    pause();

    // get or create task that is started
    int index = find_formular(task_name);

    if (index == -1)
    {
        index = forms_.size();
        forms_.push_back(EvaluationFormular(task_name, recorded));
    }
    else if (forms_[index].active)
    {
        throw RuntimeEvaluationException();
    }

    // start
    forms_[index].active = true;
    forms_[index].accumulate = 0;

    resume();
#endif
}

void RuntimeEvaluator::clear()
{
    forms_.clear();
}

void RuntimeEvaluator::stop(const std::string& task_name)
{
#ifdef TIME_MEASUREMENT
    pause();

    // get task that is stopped
    auto index = find_formular(task_name);

    if (index == -1 || !forms_[index].active)
    {
        throw RuntimeEvaluationException();
    }

    // stop
    unsigned long long time = forms_[index].accumulate;
    forms_[index].active = false;
    forms_[index].count++;
    forms_[index].last = time;
    forms_[index].sum += time;
    forms_[index].filter.update(time);
    if (time < forms_[index].min)
    {
        forms_[index].min = time;
    }
    if (time > forms_[index].max && forms_[index].count != 1) // ignore first
    {
        forms_[index].max = time;
    }

    if (forms_[index].recorded)
    {
        forms_[index].recorded_data.push_back(time);
    }

    if (task_name == "total")
    {
        double time_ms = time / 1000.0;

        if (time_ms > 50.0)
        {
            num_scans_over_50ms++;
        }

        overhang_time_ = std::max(overhang_time_ + time_ms - 50.0, 0.0);
        if (overhang_time_ >= 50.0)
        {
            num_dropped_scans++;
            overhang_time_ -= 50.0;
        }

        size_t bucket = time_ms / HIST_BUCKET_SIZE;
        bucket = std::min(bucket, histogram_.size() - 1);
        histogram_[bucket]++;
    }

    resume();
#endif
}

const std::vector<EvaluationFormular> RuntimeEvaluator::get_forms()
{
    return forms_;
}

std::string RuntimeEvaluator::to_string(bool with_recorded)
{
    pause();

    constexpr int FIELD_COUNT = 7;
    static std::string fields[FIELD_COUNT] = { "task", "count", "last", "min", "max", "avg", "run_avg" };

    size_t width = 0;
    for (const auto& ef : forms_)
    {
        width = std::max(width, ef.name.length());
    }
    for (int i = 0; i < FIELD_COUNT; i++)
    {
        width = std::max(width, fields[i].length());
    }

    std::stringstream ss;
    ss << std::setfill(' ') << "\n";
    for (int i = 0; i < FIELD_COUNT; i++)
    {
        ss << std::setw(width) << fields[i] << (i == FIELD_COUNT - 1 ? "\n" : " | ");
    }
    ss << std::setfill('-');
    for (int i = 0; i < FIELD_COUNT; i++)
    {
        ss << std::setw(width) << "" << (i == FIELD_COUNT - 1 ? "-\n" : "-+-");
    }
    ss << std::setfill(' ');

    for (const auto& ef : forms_)
    {
        if (ef.active)
        {
            continue;
        }
        unsigned long long avg = ef.sum / ef.count;
        unsigned long long run_avg = (int)ef.filter.get_mean();
        unsigned long long values[] = { ef.last, ef.min, ef.max, avg, run_avg };
        ss << std::setw(width) << ef.name << " | "
           << std::setw(width) << ef.count << " | ";
        for (int i = 0; i < FIELD_COUNT - 2; i++)
        {
            ss << std::setw(width) << values[i] / 1000 << (i == FIELD_COUNT - 3 ? "\n" : " | ");
        }
    }

    // Histogram of total time
    int total_index = find_formular("total");
    if (total_index != -1)
    {
        const auto& form = forms_[total_index];

        ss << "Over 50ms: " << std::setw(width) << num_scans_over_50ms << " / " << form.count
           << " = " << std::setw(2) << 100 * num_scans_over_50ms / form.count << "%\n";
        ss << "Dropped  : " << std::setw(width) << num_dropped_scans << " / " << form.count
           << " = " << std::setw(2) << 100 * num_dropped_scans / form.count << "%\n";

        //                  columns with padding    +    separators     -       start of line
        int line_length = FIELD_COUNT * (width + 2) + (FIELD_COUNT - 1) - std::string("10-20: ").length();

        for (size_t i = 0; i < histogram_.size(); i++)
        {
            ss << std::setw(2) << (i * HIST_BUCKET_SIZE) << '-';
            if (i < histogram_.size() - 1)
            {
                ss << std::setw(2) << (i + 1) * HIST_BUCKET_SIZE;
            }
            else
            {
                ss << "  ";
            }
            int count = std::ceil((double)histogram_[i] * line_length / form.count);
            ss << ": " << std::string(count, '=') << "\n";
        }
    }

    if (with_recorded)
    {
        ss << "\nRecorded data:\n\n";

        for (const auto& ef : forms_)
        {
            if (ef.recorded)
            {
                ss << ef.name << "\n";
                
                for (const auto& time : ef.recorded_data)
                {
                    ss << time << "\n";
                }

                ss << "\n";
            }
        }
    }

    resume();
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, RuntimeEvaluator& evaluator)
{
#ifdef TIME_MEASUREMENT
    os << evaluator.to_string();
#endif
    return os;
}

} // namespace mcl
