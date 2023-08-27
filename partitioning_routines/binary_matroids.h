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
                                                      std::vector<SubproblemSolution>& partial_solution,
                                                      const CoverageConstraint& matroid = problem.getMatroid<CoverageConstraint>())
{
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
