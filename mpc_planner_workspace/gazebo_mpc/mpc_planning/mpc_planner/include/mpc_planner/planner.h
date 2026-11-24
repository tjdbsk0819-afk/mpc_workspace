#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H

#include <mpc_planner_types/data_types.h>
#include <mpc_planner_types/module_data.h>

#include <memory>
#include <vector>
#include <map>
#include <string>

namespace RosTools
{
    class DataSaver;
    class Timer;
}

namespace MPCPlanner
{
    class RealTimeData;
    class State;
    class ControllerModule;
    class Solver;
    class ExperimentUtil;

    struct PlannerOutput
    {
        Trajectory trajectory;
        bool success{false};
        double solver_ms{0.}; // Steady-clock wall time, including failed solves.
        int solver_calls{0};

        PlannerOutput(double dt, int N) : trajectory(dt, N) {}

        PlannerOutput() = default;
    };

    class Planner
    {
    public:
        Planner();

    public:
        // Optional initial guess only: the generated OCP/cost/constraints are unchanged.
        using InitialGuess = std::vector<std::map<std::string, double>>;
        PlannerOutput solveMPC(State &state, RealTimeData &data, const InitialGuess &initial_guess = {});
        double getObjective() const;
        double getSolution(int k, std::string &&var_name) const;

        void onDataReceived(RealTimeData &data, std::string &&data_name);

        void saveData(State &state, RealTimeData &data);
        void visualize(const State &state, const RealTimeData &data);

        void reset(State &state, RealTimeData &data, bool success = true);

        bool isObjectiveReached(const State &state, const RealTimeData &data) const;

        RosTools::DataSaver &getDataSaver() const;

    private:
        bool _is_data_ready{false}, _was_reset{true};

        std::shared_ptr<Solver> _solver;
        std::shared_ptr<ExperimentUtil> _experiment_util;
        PlannerOutput _output;

        Trajectory _warmstart;

        ModuleData _module_data;

        std::unique_ptr<RosTools::Timer> _startup_timer;

        std::vector<std::shared_ptr<ControllerModule>> _modules;
    };

}

#endif
