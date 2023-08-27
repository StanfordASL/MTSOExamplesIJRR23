// @class Subproblem a subproblem type that is hashable
class Subproblem;

// @class MTSOProblem the MTSO instance to solve
class MTSOProblem;

// @class SubproblemSolution a solution for a subproblem of the MTSO
class SubproblemSolution;

// @class Subgraph A subgraph of the MTSO problem
class Subgraph;

/// Transversal Matroid: Launch constraint ///
struct LaunchConstraints
{
  size_t K;
  std::unordered_map<Edge, size_t> edge_capacity;
};

std::vector<Subproblem> partition<LaunchConstraint>(const MTSOProblem& problem,
                                                    std::vector<SubproblemSolution>& partial_solution,
                                                    const LaunchConstraint& matroid = problem.getMatroid<LaunchConstraint>())
{
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
                                                           std::vector<SubproblemSolution>& partial_solution,
                                                           const HetergeneousConstraint& matroid = problem.getMatroid<HeterogeneousConstraint>())
{
  // Check how many robots of each type we've already assigned
  std::unordered_map<RobotType, size_t> assigned_types;
  for (const SubproblemSolution& solution : partial_solutions)
  {
    assigned_types[getType(solution)]++;
  }

  std::vector<Subproblem> subproblems;
  for (const auto& [robot_type, subgraph] : matroid.subgraphs)
  {
    if (assigned_types[robot_type] < matroid.K[robot_type])
      subproblems.push_back(Subproblem{subgraph});
  }
  return subproblems;
}

/// Transversal Matroid: Risk constraint ///
struct RiskConstraint
{
  std::map<double, size_t> risk_constraints;
};
std::vector<Subproblem> partition<RiskConstraint>(const MTSOProblem& problem,
                                                  std::vector<SubproblemSolution>& partial_solution,
                                                  const RiskConstraint& matroid = problem.getMatroid<RiskConstraint>())
{
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
  for (const auto& [risk_threshold, K] : matroid.risk_constraints)
  {
    if (assigned_risks[risk_threshold] < K)
    {
      subproblems.push_back(Subproblem{problem});
      subproblems.back().setRiskThreshold(risk_threshold);
      // Returning multiple subproblems is technically fine but inefficient.
      // We only need to return the subproblem corresponding to the greatest risk threshold,
      // since this will contain the best solutions in all the other subproblems too
      break;
    }
  }
  return subproblems;
}

/// Transversal Matroid: Traffic constraint ///
struct TrafficConstraint
{
  // regions are _disjoint_ -- this is not necessary but sufficient for ensuring that all bases
  // have the same size (e.g. ensures this is a matroid versus a p-system)
  std::unordered_map<Region, Subgraph> subgraphs;
  std::unordered_map<Region, size_t> K;
};
std::vector<Subproblem> partition<TrafficConstraint>(const MTSOProblem& problem,
                                                  std::vector<SubproblemSolution>& partial_solution,
                                                  const TrafficConstraint& matroid = problem.getMatroid<TrafficConstraint>())
{
  // Check how many robots we've already assigned to a given region
  std::unordered_map<Region, size_t> assigned_region;
  for (const SubproblemSolution& solution : partial_solutions)
  {
    assigned_regions[getRegion(solution)]++;
  }

  std::vector<Subproblem> subproblems;
  for (const auto& [region, subgraph] : matroid.subgraphs)
  {
    if (assigned_regions[region] < matroid.K[region])
      subproblems.push_back(Subproblem{subgraph});
  }
  return subproblems;
}
