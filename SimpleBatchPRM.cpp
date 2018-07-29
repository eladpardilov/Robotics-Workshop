#include "SimpleBatchPRM.h"
//#include <logger.h>

#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/config/MagicConstants.h>
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/progress.hpp>

#ifndef foreach
#define foreach BOOST_FOREACH
#endif

namespace ompl
{
    namespace magic
    {

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;
    }
}


ompl::geometric::SimpleBatchPRM::SimpleBatchPRM(const base::SpaceInformationPtr &si, bool starStrategy) :
    ompl::geometric::PRM(si, starStrategy),
    useKNearest_(true),
    numNeighbors_(magic::DEFAULT_NEAREST_NEIGHBORS),
    total_num_of_samples_(0),
    numMilestones_(100)
{
    /*
     * Export set of milestones (without dealing with start / goal)
     * Build graph with milestones
     * Build graph without milestones (regenerate them)
     * Find a path in the graph
    */
    
    
    setName("SimpleBatchPRM");
    Planner::declareParam<int>("num_milestones", this, &SimpleBatchPRM::setNumMilestones, std::string("1:10000"));
    Planner::declareParam<int>("num_neighbors", this, &SimpleBatchPRM::setNumNeighbors, std::string("1:10000"));
    Planner::declareParam<std::string>("strategy_str", this, &SimpleBatchPRM::setConnectionStrategyType);

    addPlannerProgressProperty("vertex count INTEGER",
                               boost::bind(&SimpleBatchPRM::getCurrentVertexIndex, this));

    
}

ompl::geometric::SimpleBatchPRM::~SimpleBatchPRM()
{
  freeMemory();
  //si_->freeStates(allMilestones_);
}

void ompl::geometric::SimpleBatchPRM::setConnectionStrategyType(const std::string &strategy_str) {
    const char *strategy_c = strategy_str.c_str();
    if (strategy_c[0] == 'k' || strategy_c[0] == 'K') {
        useKNearest_ = true;
	if (starStrategy_ == false)
	  numNeighbors_ = boost::lexical_cast<int>(strategy_c + 1);
    } else if (strategy_c[0] == 'r' || strategy_c[0] == 'R') {
        useKNearest_ = false;
	if (starStrategy_ == false)
	  connectionRadius_ = boost::lexical_cast<double>(strategy_c + 1);
    } else {
        //LOGG_ERROR << "Unknown connection strategy: " << strategy_str;
    }
}


void ompl::geometric::SimpleBatchPRM::setup()
{

    Planner::setup();
    if (!nn_)
    {
        specs_.multithreaded = false;  // temporarily set to false since nn_ is used only in single thread
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        specs_.multithreaded = true;
        nn_->setDistanceFunction(boost::bind(&SimpleBatchPRM::distanceFunction, this, _1, _2));
    }
    if (!connectionStrategy_)
    {
        if (starStrategy_)
            connectionStrategy_ = KStarStrategy<Vertex>(
                [this]
                {
                    return milestoneCount();
                },
                nn_, si_->getStateDimension());
        else
            connectionStrategy_ = KStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS, nn_);
    }

    
    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            opt_.reset(new base::PathLengthOptimizationObjective(si_));
            if (!starStrategy_)
                opt_->setCostThreshold(opt_->infiniteCost());
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }
}



void ompl::geometric::SimpleBatchPRM::connectRoadmap(const base::PlannerTerminationCondition &ptc) {
    
    currentVertexIndex_ = 0;
    boost::progress_display show_progress(boost::num_vertices(g_), std::cerr, "Connecting Milestones...\n");
    
    foreach (Vertex m, boost::vertices(g_)) {
        currentVertexIndex_++;
        ++show_progress;

	const std::vector<Vertex>& neighbors = connectionStrategy_(m);
        
        foreach (Vertex n, neighbors) {
            
            if (ptc) 
                return;
            
            totalConnectionAttemptsProperty_[m]++;
            totalConnectionAttemptsProperty_[n]++;
            
	    bool motion_succ = si_->checkMotion(stateProperty_[m], stateProperty_[n]);
                
	    if (motion_succ) {
                    
	      successfulConnectionAttemptsProperty_[m]++;
	      successfulConnectionAttemptsProperty_[n]++;
	      const base::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
	      const Graph::edge_property_type properties(weight);
	      boost::add_edge(m, n, properties, g_);
	      uniteComponents(m, n);
	    }   
	}
    }
}


void ompl::geometric::SimpleBatchPRM::miniSetup() {
    // Remove previous variables
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();
    if (nn_)
        nn_->clear();
    
    freeMemory();
    clearQuery();
}


ompl::base::PlannerStatus ompl::geometric::SimpleBatchPRM::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    //LOGG_INFO << "SimpleBatchPRM solve";
    //LOGG_INFO << "Num milestones: " << numMilestones_;
    //if (useKNearest_)
    //    LOGG_INFO << "K-strategy with num neighbors: " << numNeighbors_;
    //else
    //    LOGG_INFO << "R-strategy with connection radius: " << connectionRadius_;
    
    miniSetup();
    
    iterations_ = 0;
    
    bestCost_ = opt_->infiniteCost();
    
    // If milestones were not imported, generate them
    if (allMilestones_.empty())
        generateMilestones();
    
    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    constructRoadmapVertices();
    
    if (startM_.empty()) {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    
    if (goalM_.empty()) {
        OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }
    
    connectRoadmap(ptc);
    
    std::pair< boost::graph_traits<Graph>::vertex_iterator, boost::graph_traits<Graph>::vertex_iterator > 
    // auto
        vp = boost::vertices(g_);
    
    OMPL_INFORM("%s: Roadmap size: %u states and %u edges", getName().c_str(), boost::num_vertices(g_), boost::num_edges(g_));
    //LOGG_INFO << "Number of connected components: " << disjointSets_.count_sets(vp.first, vp.second);
   
    // Reset addedNewSolution_ member for legacy reason
    addedNewSolution_ = false;
    
    // Search for a solution in the graph
    base::PathPtr sol;
    maybeConstructSolution(startM_, goalM_, sol);

    if (sol)
    {
        // again, for legacy reason
        addedNewSolution_ = true;
        
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, addedNewSolution());
        pdef_->addSolutionPath(psol);
    }

    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::SimpleBatchPRM::constructRoadmapVertices() {

    size_t start_milestones_index;
    size_t start_milestones_count;
    size_t goal_milestones_index;
    size_t goal_milestones_count;
    
    // Add valid start states
    start_milestones_index = allMilestones_.size();
    start_milestones_count = 0;
    
    while (const base::State *st = pis_.nextStart())
        allMilestones_.push_back(si_->cloneState(st));
    
    start_milestones_count = allMilestones_.size() - start_milestones_index;
    
    // Add valid goal states
    goal_milestones_index = allMilestones_.size();
    goal_milestones_count = 0;
    
    while (const base::State *st = pis_.nextGoal())
        allMilestones_.push_back(si_->cloneState(st));
    
    goal_milestones_count = allMilestones_.size() - goal_milestones_index;
    
    // Build graph vertices
    for (size_t i = 0; i < allMilestones_.size(); i++) {
        
        graphMutex_.lock();
        
        Vertex m = boost::add_vertex(g_);
        stateProperty_[m] = allMilestones_[i];
        
        totalConnectionAttemptsProperty_[m] = 0;
        successfulConnectionAttemptsProperty_[m] = 0;
        
        disjointSets_.make_set(m);
        
        nn_->add(m);
        
        if (start_milestones_index <= i && i < start_milestones_index + start_milestones_count)
            startM_.push_back(m);
        
        if (goal_milestones_index <= i && i < goal_milestones_index + goal_milestones_count)
            goalM_.push_back(m);

        graphMutex_.unlock();
    }
    
}

void ompl::geometric::SimpleBatchPRM::generateMilestones() {
    
    base::State *workState;
    bool found = false;
    
    //LOGG_INFO << "Generating " << numMilestones_ << " milestones...";
    
    
    allMilestones_.clear();
    allMilestones_.reserve(numMilestones_);
    
    workState = si_->allocState();
    
    while (allMilestones_.size() < (size_t)numMilestones_) {
        do {
	  //	  simpleSampler_->sampleUniform(workState);
	  //total_num_of_samples_++;
	  //found = (si_->isValid(workState));
	  found = sampler_->sample(workState);
        } while (!found);
               
        allMilestones_.push_back(si_->cloneState(workState));
    }
    
    si_->freeState(workState);
    
    //LOGG_INFO << "Built milestones. total: " << allMilestones_.size();
}

void ompl::geometric::SimpleBatchPRM::exportMilestones(std::vector<base::State *>& out) {
    out.clear();
    out.reserve(allMilestones_.size());
    
    for (const auto &st : allMilestones_) {
        out.push_back(si_->cloneState(st));
    }
}

void ompl::geometric::SimpleBatchPRM::importMilestones(const std::vector<base::State *>& in) {

    si_->freeStates(allMilestones_);
    allMilestones_.clear();
    allMilestones_.reserve(in.size());
    
    for (const auto &st : in) {
        allMilestones_.push_back(si_->cloneState(st));
    }
}

std::string ompl::geometric::SimpleBatchPRM::getCurrentVertexIndex() {
    return boost::lexical_cast<std::string>(currentVertexIndex_);
}


void ompl::geometric::SimpleBatchPRM::showLocalPlannerStatistics() {
    int total_success = 0;
    int total_attempts = 0;
    foreach (Vertex v, boost::vertices(g_)) {
        total_success += successfulConnectionAttemptsProperty_[v];
        total_attempts += totalConnectionAttemptsProperty_[v];
    }
    double success_rate = (double)total_success / total_attempts;
    
    //LOGG_INFO << "Local planner success rate: " << success_rate;    
}

double ompl::geometric::SimpleBatchPRM::computeAverageNumberOfPotentialNeighbors()
{
  if (useKNearest_){ //no need to compute average...  
    //since the strategy does not expose k we compute it once 
    //this si wastefull but...
    foreach (Vertex m, boost::vertices(g_)) {
	const std::vector<Vertex>& neighbors = connectionStrategy_(m);
	return neighbors.size();
    }
  }
  else{ //use R strategy...
    std::size_t cnt(0);
    std::size_t sum(0);

    foreach (Vertex m, boost::vertices(g_)) {
	const std::vector<Vertex>& neighbors = connectionStrategy_(m);
	cnt++;
	sum+=neighbors.size();
    }
    return double(sum) / double(cnt);
  }
  return 0; //dummy
}

double  ompl::geometric::SimpleBatchPRM::estimateCspaceRelativeVolume(std::size_t n)
{
  ompl::base::StateSamplerPtr sampler = si_->getStateSpace()->allocDefaultStateSampler();
  std::size_t free(0);
  base::State *workState = si_->allocState();
  for (std::size_t i(0); i < n; ++i){
    sampler->sampleUniform(workState);
    if (si_->isValid(workState))
      free++;
  }

  return double(free) / double(n);

}
