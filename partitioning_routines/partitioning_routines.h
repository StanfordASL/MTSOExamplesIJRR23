// @class Subproblem a subproblem type that is hashable
class Subproblem;

// @class MTSOProblem the MTSO instance to solve
class MTSOProblem;

// @class SubproblemSolution a solution for a subproblem of the MTSO
class SubproblemSolution;

// @class Subgraph A subgraph of the MTSO problem
class Subgraph;

// All matroids must provide an implementation of the partition routine

template <class Matroid>
std::vector<Subproblem> partition(const MTSOProblem& problem, std::vector<SubproblemSolution>& partial_solution);

/// Uniform Matroid: Carinality constraint  ///
/**
 * @struct CardinalityConstraint
 * Enforces there are no more than K agents. This is an instance of a uniform matroid
 */
struct CardinalityConstraint
{
  size_t K;
};
std::vector<Subproblem> partition<CardinalityConstraint>(const MTSOProblem& problem,
                                                         std::vector<SubproblemSolution>& partial_solution)
{
  const CardinalityConstraint& matroid = problem.getMatroid<CardinalityConstraint>();

  // Check if the solution is full rank
  if (partial_solution.size() == matroid.K)
    return {};

  // Otherwise form a subproblem out of the parent problem
  return {Subproblem(problem)};
}

/// Binary Matroid: Coverage constraint  ///
// @brief getFocus Returns the subgraph that is the focus of this solution. Ties are broken deterministically
const Subgraph& getFocus(const SubproblemSolution& solution, const std::vector<SubGraph>& subgraphs);

/**
 * @class CoverageConstraint
 * Enforces each path 'focuses' on a different subgraph of the problem.
 */
struct CoverageConstraint
{
  size_t K;
  std::vector<SubGraph> subgraphs;
};

std::vector<Subproblem> partition<CoverageConstraint>(const MTSOProblem& problem,
                                                      std::vector<SubproblemSolution>& partial_solution)
{
  const CoverageConstraint& matroid = problem.getMatroid<CoverageConstraint>();

  // Check if the solution is full rank
  if (partial_solution.size() == matroid.K || partial_solution.size() == matroid.subgraphs.size())
    return {};

  // @note - this loop is written for clarity and evaluates getFocus more than necessary
  std::vector<Subproblem> subproblems;
  for (const SubGraph& subgraph : matroid.subgraphs)
  {
    bool subgraph_in_focus = false;
    for (const SubproblemSolution& solution : partial_solutions)
    {
      if (getFocus(solution) == subgraph)
      {
        subgraph_in_focus = true;
        break;
      }
    }
    if (!subgraph_in_focus)
      subproblems.push_back(Subproblem{subgraph});
  }
  return subproblems;
}

/// Transversal Matroid: Launch constraint ///
struct LaunchConstraints
{
  size_t K;
  std::unordered_map<Edge, size_t> edge_capacity;
};

std::vector<Subproblem> partition<LaunchConstraint>(const MTSOProblem& problem,
                                                    std::vector<SubproblemSolution>& partial_solution)
{
  const LaunchConstraint& matroid = problem.getMatroid<LaunchConstraint>();

  // Check if the solution is full rank
  if (partial_solution.size() == matroid.K)
    return {};

  // Create the subproblem out of the base problem
  std::vector<Subproblem> subproblems(Subproblem{problem});

  // Calculate capacity of each edge. This can be cached for efficiency
  std::unordered_map<Edge, size_t> edge_traffic;
  for (const SubproblemSolution& solution : partial_solution)
  {
    for (const Edge& edge : solution)
      edge_traffic[edge]++;
  }

  // Remove any edges that are at capacity
  for (const[edge, traffic] & : edge_traffic)
  {
    if (traffic == matroid.edge_capacity.at(edge))
      subproblems.front().removeEdge(edge);
  }

  return subproblems;
}

/// Transversal Matroid: Heterogeneous constraint ///

class RobotType;
RobotType getType(const SubproblemSolution& solution);

struct HeterogeneousConstraint
{
  std::unordered_map<RobotType, Subgraph> subgraphs;
  std::unordered_map<RobotType, size_t> K;
};
std::vector<Subproblem> partition<HeterogeneousConstraint>(const MTSOProblem& problem,
                                                           std::vector<SubproblemSolution>& partial_solution)
{
  const HetergeneousConstraint& matroid = problem.getMatroid<HeterogeneousConstraint>();

  // Check how many robots of each type we've already assigned
  std::unordered_map<RobotType, size_t> assigned_types;
  for (const SubproblemSolution& solution : partial_solutions)
  {
    assigned_types[getType(solution)]++;
  }

  std::vector<Subproblem> subproblems;
  for (const[robot_type, subgraph] & : matroid.subgraphs)
  {
    if (assigned_types[robot_type] < matroid.K[robot_type])
      subproblems.push_back(Subproblem{subgraph});
  }
  return subproblems;
}

/// Transversal Matroid: Traffic constraint -- TODO///
struct TrafficConstraint
{
};

/// Transversal Matroid: Risk constraint ///
struct RiskConstraint
{
  std::map<double, size_t> risk_constraints;
};
std::vector<Subproblem> partition<RiskConstraint>(const MTSOProblem& problem,
                                                  std::vector<SubproblemSolution>& partial_solution)
{
  const RiskConstraint& matroid = problem.getMatroid<RiskConstraint>();

  // Check how many robots of each risk type we've already assigned
  std::map<double, size_t> assigned_risks;
  for (const SubproblemSolution& solution : partial_solutions)
  {
    double risk = getRisk(solution);
    for (const[risk_threshold, _] & : matroid.risk_constraints)
    {
      // Because  risk_constraints is sorted, we can stop after the first viable assignment is found
      if (risk < risk_threshold)
      {
        assigned_risks[risk_threshold]++;
        break;
      }
    }
  }

  std::vector<Subproblem> subproblems;
  for (const[risk_threshold, K] & : matroid.subgraphs)
  {
    if (assigned_risks[risk_threshold] < K)
    {
      subproblems.push_back(Subproblem{problem});
      subproblems.back().setRiskThreshold(risk_threshold);
    }
  }
  return subproblems;
}

/// Gammoid (Laminar Matroid): Nested cardinality constraints ///

// Make this simply the regions + ps_vals + robot_types matroids. Put off because this can take a while

/// @struct Trait - representation of a trait based on group and value. Traits could represent things such as risk
/// group, robot type, etc.
struct Trait
{
  size_t trait_id;
  size_t trait_value;
};
size_t getTraitValue(size_t trait_id, const MTSOProblem& problem, const SubproblemSolution& solution);

struct NestedConstraint
{
  std::unordered_map<Trait, size_t> trait_capacity;

  TraitGraph
};

std::vector<Subproblem> partition<NestedConstraint>(const MTSOProblem& problem,
                                                    std::vector<SubproblemSolution>& partial_solution)
{
  const NestedConstraint& matroid = problem.getMatroid<NestedConstraint>();

  // This algorithm needs to do a depth first search over trait types
  // TODO:

  return subproblems;
}

/// Matroid Truncation ///
template <struct UnderlyingMatroid>
struct MatroidTruncation
{
  size_t K;
  UnderlyingMatroid matroid;
};

template <struct UnderlyingMatroid>
std::vector<Subproblem> partition<struct Truncation<UnderlyingMatroid>>(
    const MTSOProblem& problem,
    std::vector<SubproblemSolution>& partial_solution,
    const Truncation<UnderlyingMatroid>& matroid = problem.getMatroid<Truncation<UnderlyingMatroid>>())
{
  if (partial_soution.size() == matroid.K)
    return {};
  return partition<UnderlyingMatroid>(problem, partial_solution, matroid.matroid);
}
