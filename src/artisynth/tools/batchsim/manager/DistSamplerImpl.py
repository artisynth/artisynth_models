"""
An implementation of a DistributionSampler in Jython so that it can use
code from the jdistlib library without having compiler errors if that library
is not present on a particular ArtiSynth user's system (jdistlib does not
ship with ArtiSynth due to license incompatibilities). The website for
jdistlib is http://jdistlib.sourceforge.net/.

@author: Francois Roewer-Despres
"""

from artisynth.tools.batchsim.manager import DistributionSampler
from jarray import array
from jdistlib import *
from jdistlib.evd import *
from jdistlib.rng import MersenneTwister

# See the jdistlib javadocs for additional details on each distribution:
# http://jdistlib.sourceforge.net/javadoc/
# Note that some distributions are not supported.
distSpecs = {
    "Ansari" : [2, Ansari],  # Ansari(int m, int n)
    "Arcsine" : [2, Arcsine],  # Arcsine(double a, double b)
    "Beta" : [2, Beta],  # Beta(double a, double b)
    "BetaBinomial" : [3, BetaBinomial],  # BetaBinomial(double mu, double sigma, int bd)
    "BetaPrime" : [2, BetaPrime],  # BetaPrime(double a, double b)
    "Binomial" : [2, Binomial],  # Binomial(double n, double p)
    "Cauchy" : [2, Cauchy],  # Cauchy(double location, double scale)
    "Chi" : [1, Chi],  # Chi(double df)
    "ChiSquare" : [1, ChiSquare],  # ChiSquare(double df)
    "Exponential" : [1, Exponential],  # Exponential(double scale)
    "F" : [2, F],  # F(double df1, double df2)
    "Fretchet" : [3, Fretchet],  # Fretchet(double loc, double scale, double shape)
    "Gamma" : [2, Gamma],  # Gamma(double shape, double scale)
    "GeneralizedPareto" : [3, GeneralizedPareto],  # GeneralizedPareto(double loc, double scale, double shape)
    "Geometric" : [1, Geometric],  # Geometric(double p)
    "GEV" : [3, GEV],  # GEV(double loc, double scale, double shape)
    "Gumbel" : [2, Gumbel],  # Gumbel(double loc, double scale)
    "HyperGeometric" : [3, HyperGeometric],  # HyperGeometric(double r, double b, double n)
    "InvGamma" : [2, InvGamma],  # InvGamma(double shape, double scale)
    "InvNormal" : [2, InvNormal],  # InvNormal(double mu, double sigma)
    "Kendall" : [1, Kendall],  # Kendall(int n)
    "Kumaraswamy" : [2, Kumaraswamy],  # Kumaraswamy(double a, double b)
    "Laplace" : [2, Laplace],  # Laplace(double location, double scale)
    "Logarithmic" : [1, Logarithmic],  # Logarithmic(double mu)
    "Logistic" : [2, Logistic],  # Logistic(double location, double scale)
    "LogNormal" : [2, LogNormal],  # LogNormal(double meanlog, double sdlog)
    "Nakagami" : [2, Nakagami],  # Nakagami(double m, double omega)
    "NegBinomial" : [2, NegBinomial],  # NegBinomial(double size, double prob)
    "NonCentralBeta" : [3, NonCentralBeta],  # NonCentralBeta(double a, double b, double ncp)
    "NonCentralChiSquare" : [2, NonCentralChiSquare],  # NonCentralChiSquare(double df, double ncp)
    "NonCentralF" : [3, NonCentralF],  # NonCentralF(double df1, double df2, double ncp)
    "NonCentralT" : [2, NonCentralT],  # NonCentralT(double df, double ncp)
    "Normal" : [2, Normal],  # Normal(double mu, double sigma)
    "Poisson" : [1, Poisson],  # Poisson(double lambda)
    "Rayleigh" : [1, Rayleigh],  # Rayleigh(double scale)
    "ReverseWeibull" : [3, ReverseWeibull],  # ReverseWeibull(double loc, double scale, double shape)
    "SignRank" : [1, SignRank],  # SignRank(int n)
    "Spearman" : [1, Spearman],  # Spearman(int n)
    "T" : [1, T],  # T(double df)
    "Tukey" : [3, Tukey],  # Tukey(double rr, double cc, double df)
    "Uniform" : [2, Uniform],  # Uniform(double a, double b)
    "Weibull" : [2, Weibull],  # Weibull(double shape, double scale)
    "Zipf" : [2, Zipf]  # Zipf(int N, double s)
}

class CategoricalDistribution:
    def __init__(self, pmf):
        self._RND_ENG = None
        self._members = []
        self._probs = []
        for member in pmf.keySet():
            self._members.append(member)
            self._probs.append(pmf.get(member))

    def setRandomEngine(self, rand):
        self._RND_ENG = rand

    def random(self):
        val = self._RND_ENG.nextDouble(True, True)
        lower = 0
        upper = lower + self._probs[0]
        for i in range(1, len(self._probs)):
            if lower <= val and val < upper:
                return self._members[i-1]
            else:
                lower = upper
                upper = lower + self._probs[i]
        return self._members[-1]


class DistSamplerImpl(DistributionSampler):
    def __init__(self):
        self._RND_ENG = MersenneTwister()
        self._id2Dist = {}
        self._count = 0

    def setSeed(self, seed):
        self._RND_ENG.setSeed(seed)

    def addDistribution(self, dist, params):
        try:
            numParams, ctor = distSpecs[dist.toString()]
        except:
            raise
        if numParams != params.size():
            raise
        paramsCopy = []
        for i in range(0, params.size()):
            paramsCopy.append(self._coerce(params.get(i)))
        try:
            trueDist = ctor(*paramsCopy)
        except:
            raise
        return self._addDist(trueDist, dist.isDiscrete())

    def addCategoricalDistribution(self, pmf):
        # Categorical has discrete support, but NOT necessarily integer-based.
        return self._addDist(CategoricalDistribution(pmf), False)

    def _coerce(self, x):
        if int(x) == x:
            return int(x)
        else:
            return x

    def _addDist(self, trueDist, discrete):
        trueDist.setRandomEngine(self._RND_ENG)
        self._id2Dist[self._count] = (trueDist, discrete)
        ret = self._count
        self._count = self._count + 1
        return ret

    def sample(self, distributionIdentifier):
        try:
            trueDist, discrete = self._id2Dist[distributionIdentifier]
            if discrete:
                return int(trueDist.random())
            else:
                return trueDist.random()
        except:
            raise

