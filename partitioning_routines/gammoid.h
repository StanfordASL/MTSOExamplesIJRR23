// @class Subproblem a subproblem type that is hashable
class Subproblem;

// @class MTSOProblem the MTSO instance to solve
class MTSOProblem;

// @class SubproblemSolution a solution for a subproblem of the MTSO
class SubproblemSolution;

// @class Subgraph A subgraph of the MTSO problem
class Subgraph;


/// Gammoid (Laminar Matroid): Nested cardinality constraints ///
//TODO: ETA 9/23

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

  return subproblems;
}
