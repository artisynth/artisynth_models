" Vim syntax file.
" Language: BatchSim's Property Specification Language (PSL)
" Maintainer: Francois Roewer-Despres
" Latest Revision: July 16 2017

if exists("b:current_syntax")
    finish
endif

syn keyword pslTodo contained TODO FIXME XXX NOTE
syn match pslComment "#.*$" contains=pslTodo

syn match pslJythonDelim "\$"

syn keyword pslStatements redef when skip jython return_value get end
set iskeyword+=@-@
syn keyword pslDecorators @COMB @PROB @PHONY

syn keyword pslDist Ansari Arcsine Beta BetaBinomial BetaPrime
syn keyword pslDist Binomial Cauchy Chi ChiSquare Exponential F
syn keyword pslDist Fretchet GammaGeneralizedPareto Geometric GEV
syn keyword pslDist Gumbel HyperGeometric InvGamma InvNormal
syn keyword pslDist Kendall Kumaraswamy Laplace Logarithmic
syn keyword pslDist Logistic LogNormal Nakagami NegBinomial
syn keyword pslDist NonCentralBeta NonCentralChiSquare
syn keyword pslDist NonCentralF NonCentralT Normal Poisson
syn keyword pslDist Rayleigh ReverseWeibull SignRank Spearman T
syn keyword pslDist Tukey Uniform Weibull Zipf

syn match tildeOrEqual "=\|\~"

syn region pslString start='"' end='"'
syn region pslValue start='%' end='%'

syn match pslNumber "[-+]\?\(\d\+\.\|\d*\.\?\d\+\)\([eE][-+]\?\d\+\)\?"

let b:current_syntax = "psl"

hi def link pslTodo        Todo
hi def link pslJythonDelim Todo
hi def link pslComment     Comment
hi def link pslStatements  Statement
hi def link pslDecorators  PreProc
hi def link pslDist        Type
hi def link tildeOrEqual   SpecialChar
hi def link pslString      Identifier
hi def link pslValue       Special
hi def link pslNumber      Number
