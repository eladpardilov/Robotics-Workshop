#ifndef OMPL_GEOMETRIC_PLANNERS_SIMPLE_BATCH_PRM_
#define OMPL_GEOMETRIC_PLANNERS_SIMPLE_BATCH_PRM_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include "DirectedPRM.h"
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>


#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <utility>
#include <vector>
#include <map>
#include <chrono>

namespace ompl
{

    namespace base
    {
        // Forward declare for use in implementation
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {

        /** \brief Probabilistic RoadMap planner for Aviel's experiments */
        class SimpleBatchPRM : public DirectedPRM
        {
        public:

            /** \brief Constructor */
            SimpleBatchPRM(const base::SpaceInformationPtr &si, bool starStrategy = false);

            virtual ~SimpleBatchPRM();

            
	        /** \brief Function that can solve the motion planning
                problem. Grows a roadmap using
                constructRoadmap(). This function can be called
                multiple times on the same problem, without calling
                clear() in between. This allows the planner to
                continue work for more time on an unsolved problem,
                for example. Start and goal states from the currently
                specified ProblemDefinition are cached. This means
                that between calls to solve(), input states are only
                added, not removed. When using PRM as a multi-query
                planner, the input states should be however cleared,
                without clearing the roadmap itself. This can be done
                using the clearQuery() function. */
            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);
            
            void constructRoadmapVertices(int current_milestones);

            void connectRoadmap(const base::PlannerTerminationCondition &ptc);

            void getPlannerData(base::PlannerData &data) const override;

            /* Custom optimization objective function, combinine euclidian distance and angle differences */
            ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si);

                        
           
	public:

            void miniSetup();
            
            void setNumMilestones(int num_milestones) { numMilestones_ = num_milestones; }

            void setNumNeighbors(int num_neighbors) { numNeighbors_ = num_neighbors; }

            void setConnectionRadius(double r) { connectionRadius_ = r;}
            
	    //set connection strategy: either 'K17' or 'R0.1'
            void setConnectionStrategyType(const std::string &strategy_str);

	protected: //functions / params related to coonection strategy    
	    //flag to indicate what connection strategy is used
            bool useKNearest_;

	    //value of connection strategy (either K or R)
	    //applicable only when star strategy is not used
            int numNeighbors_; //value of k
            double connectionRadius_; //value of r
	
	    // either 'K17' or 'R0.1'
            std::string strategyStr_;

            //void setFreespaceRatio(double XfreeRatio);
	    int total_num_of_samples_;

	protected: //mileston-related params / functions
	    std::vector<base::State *> allMilestones_;
            int numMilestones_;
            int currentVertexIndex_; //ToDo: add support
            
	public:  // My functions
            void generateMilestones();

            void generateExtraMilestones();
            
            void exportMilestones(std::vector<base::State *>& out);
            
            void importMilestones(const std::vector<base::State *>& in);

	    std::string getCurrentVertexIndex();

            void showLocalPlannerStatistics();

	    double computeAverageNumberOfPotentialNeighbors();

	    double estimateCspaceRelativeVolume(std::size_t n);
        };
    }
}

#endif