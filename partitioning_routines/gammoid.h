// @class Subproblem a subproblem type that is hashable
class Subproblem;

// @class MTSOProblem the MTSO instance to solve
class MTSOProblem;

// @class SubproblemSolution a solution for a subproblem of the MTSO
class SubproblemSolution;

// @class Subgraph A subgraph of the MTSO problem
class Subgraph;


/// Gammoid (Laminar Matroid): Nested cardinality constraints ///

struct NestedConstraint
{
  // layer 0: truncation
  size_t K;
  // layer 1: Risk
  std::map<double, size_t> risk_constraints;
  // layer 2: Robot type and subgraphs
  std::unordered_map<RobotType, size_t> type_constraints;
  std::unordered_map<RobotType, Subgraph> type_subgraphs;
  // layer 3: Region type and subgraphs
  std::unordered_map<Region, size_t> region_constraints;
  std::unordered_map<Region, Subgraph> region_subgraphs;
};

std::vector<Subproblem> partition<NestedConstraint>(const MTSOProblem& problem,
                                                    std::vector<SubproblemSolution>& partial_solution)
{
  const NestedConstraint& matroid = problem.getMatroid<NestedConstraint>();

  if(partial_solution.size() >= K)
    return {};

  // For each layer of our constraints we do the following:
  // - For each subproblem with capacity
  //   - restrict the existing solutions to the subproblem
  //   - evaluate the next layer
  // We put everything in one loop here for ease of presentation, but in practice you would split these out


  // Look for capacity in assigned risk values
  std::map<double, size_t> assigned_risks;
  for (const SubproblemSolution& solution : partial_solutions)
  {
    double risk = getRisk(solution);
    for (const[risk_threshold, _] & : matroid.risk_constraints)
    {
      // Because risk_constraints is sorted, we can stop after the first viable assignment is found
      if (risk < risk_threshold)
      {
        assigned_risks[risk_threshold]++;
        break;
      }
    }
  }


  // Form subproblems for each viable value of the outer layer:
  for(const auto& [risk_threshold, K] : matroid.risk_constraints)
  {
    // check if this layer is already full
    if(assigned_risks[risk_threshold] >= K)
      continue;

    // Within this layer we want to only compare against solutions that are in this layer
    // This is the important distinction between a gammoid and a matroid intersection
    std::vector<SubproblemSolution> layer_1_solutions;
    for(const SubproblemSolution& solution : partial_solutions)
    {
      if(getRisk(solution) > risk_threshold)
        continue;
      layer_1_solutions.push_back(solution);
    }


    // Now search for space in the robot types
    std::unordered_map<RobotType, size_t> assigned_types;
    for(const SubproblemSolution& solution : layer_1_solutions)
    {
      assigned_types[getType(solution)]++;
    }

    for (const[robot_type, type_subgraph] & : matroid.type_subgraphs)
    {
      //check if this layer is already full
      if (assigned_types[robot_type] >= matroid.type_constraints[robot_type])
        continue;

      // Extract solutions relevant for this layer and continue
      std::vector<SubproblemSolution> layer_2_solutions;
      for(const SubproblemSolution& solution : layer_1_solutions)
      {
        if(getType(solution) == robot_type)
          layer_2_solutions.push_back(solution);
      }

      // Evaluate layer 3 (traffic constraints):
      std::unordered_map<Region, size_t> assigned_regions;
      for(const SubproblemSolution& solution : layer_2_solutions)
      {
        assigned_regions[getRegion(solution)]++;
      }

      for(const auto[region, region_subgraph]& : matroid.region_subgraphs)
      {
        // check if this layer is already full
        if(assigned_regions[region] >= matroid.region_constraints[region])
          continue;

        // Create a subgraph that is the intersection of regions and types
        Subgraph intersection = intersection(type_subgraph, region_subgraph);
        subproblems.push_back(Subproblem{risk_threshold, robot_type, intersection(type_subgraph, region_subgraph)});
      }
    }
  }
  return subproblems;
}
