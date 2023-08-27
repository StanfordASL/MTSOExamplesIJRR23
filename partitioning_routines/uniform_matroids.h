// @class Subproblem a subproblem type that is hashable
class Subproblem;

// @class MTSOProblem the MTSO instance to solve
class MTSOProblem;

// @class SubproblemSolution a solution for a subproblem of the MTSO
class SubproblemSolution;

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
                                                         std::vector<SubproblemSolution>& partial_solution,
                                                         const CardinalityConstraint& matroid = problem.getMatroid<CardinalityConstraint>())
{
  // Check if the solution is full rank
  if (partial_solution.size() == matroid.K)
    return {};

  // Otherwise form a subproblem out of the parent problem
  return {Subproblem(problem)};
}

