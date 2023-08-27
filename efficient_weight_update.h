/**
 * @brief Calculates the multilinear extension using Algorithm 5
 *
 * @param [in]  value   The value of a product-form function
 * @param [in]  y       Probability the path is in set
 *
 */
double UpdateWeights(std::vector<double> value, std::vector<double> y)
{
  assert(value.size() == y.size());
  // Note: value in the context of Algorithm 5 is 1 - E[z_j(x)] which is the probability a node has not been visited
  // we avoid using that term here to highlight the generality of this function (works for any product form function)

  size_t L = y.size();

  // Initialize w[n] to be the probability that n events happened
  // Before any updates w[0] = 1 and w[j] = 0 for j > 0
  std::vector<double> w(L + 1, 0);
  w[0] = 1.0;

  // Now we update w by evaluating the probability each event might happen
  for(int j = 0; j < w.size(); j++)
  {
    // The probability m events occur is:
    //   the probability (m-1 events have already occurred and a new one occurs)
    // + the probability (that m events have already occurred and no new one occurs)
    double p_new_event = y[j];
    double p_no_new_event = 1-p_new_event;

    // We're interested in calculating the expected value of a product-form function. So if the event occurs, we include the respective term
    double value_this_event = value[j];

    // Apply the update in-place to calculate the expected value
    for(size_t m = L; m > 0; m--)
      w[m] = p_new_event * value_this_event * w[m-1] + p_no_new_event * w[m];
    w[0] *= p_no_new_event;
  }

  // Since we are interested in the expected value we need to reduce to a single value
  double expected_value = 0;
  for(int j = 0; j < w.size(); j++)
   expected_value+= w[j];
  return expected_value;
}
