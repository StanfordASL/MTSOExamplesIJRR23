// @class Subproblem a subproblem type that is hashable
class Subproblem;

// @class MTSOProblem the MTSO instance to solve
class MTSOProblem;

// @class SubproblemSolution a solution for a subproblem of the MTSO
class SubproblemSolution;

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
