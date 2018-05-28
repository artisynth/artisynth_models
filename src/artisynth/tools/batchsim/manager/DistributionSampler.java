package artisynth.tools.batchsim.manager;

import java.util.List;
import java.util.Map;

/**
 * Interface to a {@code Jython} class ({@code DistSamplerImpl.py}) that uses
 * {@code jdistlib} to sample probability distributions. Having the
 * implementation in {@code Jython} allows the code to be committed to the
 * {@code artisynth_models} repository without causing compiler errors, since
 * the {@code jdistlib} library can't be distributed with {@code ArtiSynth} due
 * to licensing issues. To use probabilistic simulations in the
 * {@link artisynth.tools.batchsim Batch Simulation Framework}, please
 * <a href="http://jdistlib.sourceforge.net/">download jdistlib</a>, and put the
 * resulting jar somewhere on the build path or class path, for example in
 * {@code artisynth_core/lib/}.
 * 
 * @author Francois Roewer-Despres
 */
public interface DistributionSampler {

   /**
    * See the <a href="http://jdistlib.sourceforge.net/javadoc/">jdistlib
    * Javadocs</a> for additional details on each distribution. Note that some
    * distributions are not supported.
    * 
    * @author Francois Roewer-Despres
    */
   public enum Distribution {
      /** <code>Ansari(int m, int n)</code> */
      Ansari (2),
      /** <code>Arcsine(double a, double b)</code> */
      Arcsine (2),
      /** <code>Beta(double a, double b)</code> */
      Beta (2),
      /** <code>BetaBinomial(double mu, double sigma, int bd)</code> */
      BetaBinomial (3, true),
      /** <code>BetaPrime(double a, double b)</code> */
      BetaPrime (2),
      /** <code>Binomial(double n, double p)</code> */
      Binomial (2, true),
      /** <code>Cauchy(double location, double scale)</code> */
      Cauchy (2),
      /** <code>Chi(double df)</code> */
      Chi (1),
      /** <code>ChiSquare(double df)</code> */
      ChiSquare (1),
      /** <code>Exponential(double scale)</code> */
      Exponential (1),
      /** <code>F(double df1, double df2)</code> */
      F (2),
      /** <code>Fretchet(double loc, double scale, double shape)</code> */
      Fretchet (3),
      /** <code>Gamma(double shape, double scale)</code> */
      Gamma (2),
      /** <code>GeneralizedPareto(double loc, double scale, double shape)</code> */
      GeneralizedPareto (3),
      /** <code>Geometric(double p)</code> */
      Geometric (1, true),
      /** <code>GEV(double loc, double scale, double shape)</code> */
      GEV (3),
      /** <code>Gumbel(double loc, double scale)</code> */
      Gumbel (2),
      /** <code>HyperGeometric(double r, double b, double n)</code> */
      HyperGeometric (3, true),
      /** <code>InvGamma(double shape, double scale)</code> */
      InvGamma (2),
      /** <code>InvNormal(double mu, double sigma)</code> */
      InvNormal (2),
      /** <code>Kendall(int n)</code> */
      Kendall (1),
      /** <code>Kumaraswamy(double a, double b)</code> */
      Kumaraswamy (2),
      /** <code>Laplace(double location, double scale)</code> */
      Laplace (2),
      /** <code>Logarithmic(double mu)</code> */
      Logarithmic (1, true),
      /** <code>Logistic(double location, double scale)</code> */
      Logistic (2),
      /** <code>LogNormal(double meanlog, double sdlog)</code> */
      LogNormal (2),
      /** <code>Nakagami(double m, double omega)</code> */
      Nakagami (2),
      /** <code>NegBinomial(double size, double prob)</code> */
      NegBinomial (2, true),
      /** <code>NonCentralBeta(double a, double b, double ncp)</code> */
      NonCentralBeta (3),
      /** <code>NonCentralChiSquare(double df, double ncp)</code> */
      NonCentralChiSquare (2),
      /** <code>NonCentralF(double df1, double df2, double ncp)</code> */
      NonCentralF (3),
      /** <code>NonCentralT(double df, double ncp)</code> */
      NonCentralT (2),
      /** <code>Normal(double mu, double sigma)</code> */
      Normal (2),
      /** <code>Poisson(double lambda)</code> */
      Poisson (1, true),
      /** <code>Rayleigh(double scale)</code> */
      Rayleigh (1),
      /** <code>ReverseWeibull(double loc, double scale, double shape)</code> */
      ReverseWeibull (3),
      /** <code>SignRank(int n)</code> */
      SignRank (1, true),
      /** <code>Spearman(int n)</code> */
      Spearman (1),
      /** <code>T(double df)</code> */
      T (1),
      /** <code>Tukey(double rr, double cc, double df)</code> */
      Tukey (3),
      /** <code>Uniform(double a, double b)</code> */
      Uniform (2),
      /** <code>Weibull(double shape, double scale)</code> */
      Weibull (2),
      /** <code>Zipf(int N, double s)</code> */
      Zipf (2, true);

      private int nparams;
      private boolean discrete = false;

      private Distribution (int numParams) {
         nparams = numParams;
      }

      private Distribution (int numParams, boolean isDiscrete) {
         nparams = numParams;
         discrete = isDiscrete;
      }

      /**
       * Returns the number of parameters of this parametric probability
       * distribution.
       * 
       * @return the number of parameters of this probability distribution
       */
      public int numParams () {
         return nparams;
      }

      /**
       * Returns {@code true} if this parametric probability distribution has a
       * discrete (i.e. INTEGER) support.
       * 
       * @return whether the support of this probability distribution is
       * discrete/integral
       */
      public boolean isDiscrete () {
         return discrete;
      }
   }

   /**
    * (Optional) Sets the seed of this {@link DistributionSampler}'s random
    * number generator to the given seed.
    *
    * @param seed
    * the seed of the random number generator
    */
   void setSeed (int seed);

   /**
    * Adds an instance of the given distribution instantiated with the given
    * parameters, and returns a unique distribution identifier that can be used
    * as a handle to the newly added distribution (analogous to file descriptors
    * in C).
    * 
    * @param dist
    * the parametric probability distribution to instantiate and add
    * @param params
    * a list containing the value for each of the distribution's parameters, in
    * order
    * @return a unique distribution identifier
    * @throws IllegalArgumentException
    * if {@code params.size() != dist.numParams()}, or the value of any of the
    * given parameters is illegal for the given distribution (e.g. requires <b>
    * {@code int}</b>, but <b>{@code double}</b> was given, or requires
    * non-negative <b>{@code double}</b>, but negative <b>{@code double}</b> was
    * given)
    */
   int addDistribution (Distribution dist, List<Double> params)
      throws IllegalArgumentException;

   /**
    * Adds an instance of a <a href=
    * "https://en.wikipedia.org/wiki/Categorical_distribution">Categorical
    * Distribution</a> defined explicitly by the given probability mass
    * function, and returns a unique distribution identifier that can be used as
    * a handle to the newly added distribution (analogous to file descriptors in
    * C).
    *
    * @param pmf
    * the probability mass function defining the distribution; this is a mapping
    * between a given member of the category and the probability of observing
    * that member
    * @return a unique distribution identifier
    */
   int addCategoricalDistribution (Map<String,Double> pmf);

   /**
    * Samples the distribution that was previously added and whose distribution
    * identifier is {@code distributionIdentifier}.
    * 
    * @param distributionIdentifier
    * the identifier of the distribution to sample
    * @return a single sample from the distribution
    * @throws IllegalArgumentException
    * if {@code distributionIdentifier} is invalid
    * @see #addDistribution(Distribution, List)
    */
   Object sample (int distributionIdentifier) throws IllegalArgumentException;
}
