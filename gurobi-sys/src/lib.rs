#![allow(improper_ctypes)]
#![allow(non_camel_case_types)]

pub use std::os::raw::{c_int, c_double, c_char, c_void};
pub type c_str = *const c_char;

use std::ffi::CString;
use std::convert::From;

#[repr(C)]
pub struct GRBenv;

#[repr(C)]
pub struct GRBmodel;

#[repr(C)]
pub struct GRBsvec {
  /// sparse vector length
  pub len: c_int,
  /// indices array of the sparse vector
  pub ind: *mut c_int,
  /// value array of the sparse vector
  pub val: *mut c_double
}


#[derive(Debug,Copy,Clone)]
pub enum IntParam {
  AggFill,
  Aggregate,
  BarCorrectors,
  BarHomogeneous,
  BarIterLimit,
  BarOrder,
  BQPCuts,
  BranchDir,
  CliqueCuts,
  ConcurrentJobs,
  ConcurrentMIP,
  CoverCuts,
  Crossover,
  CrossoverBasis,
  CSBatchMode,
  CSClientLog,
  CSIdleTimeout,
  CSPriority,
  CSTLSInsecure,
  CutAggPasses,
  CutPasses,
  Cuts,
  DAT_0265e4c4,
  DAT_026692d4,
  DAT_02669518,
  DegenMoves,
  Disconnected,
  DisplayInterval,
  DistributedMIPJobs,
  DualReductions,
  FlowCoverCuts,
  FlowPathCuts,
  FuncPieces,
  GomoryPasses,
  GUBCoverCuts,
  GURO_PAR_ADAPTCONSBAS,
  GURO_PAR_ALTSCALE,
  GURO_PAR_BARDENSECOUNT,
  GURO_PAR_BARDENSEGS,
  GURO_PAR_BARDENSESEP,
  GURO_PAR_BARDENSETHRESH,
  GURO_PAR_BARLOOSE,
  GURO_PAR_BARQUICKSTART,
  GURO_PAR_BARSTART,
  GURO_PAR_BARUNBD,
  GURO_PAR_CLOSECUTS,
  GURO_PAR_CROSSOVERLU,
  GURO_PAR_CROSSOVERPI,
  GURO_PAR_CUTBASIS,
  GURO_PAR_CUTLIFT,
  GURO_PAR_DISJOINT,
  GURO_PAR_DISTRIBUTED,
  GURO_PAR_DISTRIBUTEDMODE,
  GURO_PAR_DUMP,
  GURO_PAR_EXACTNORM,
  GURO_PAR_FEASHELPER,
  GURO_PAR_FIXEDCHARGE,
  GURO_PAR_FORCEHEUR,
  GURO_PAR_FORCEQP,
  GURO_PAR_FORCESOCP,
  GURO_PAR_GRBCUT1,
  GURO_PAR_GREEDYHEUR,
  GURO_PAR_GUIDEDPRE,
  GURO_PAR_IMPLICITEQ,
  GURO_PAR_IMPLIEDINT,
  GURO_PAR_INFPROOF,
  GURO_PAR_INFPROP,
  GURO_PAR_INTBASIS,
  GURO_PAR_ISVEXPIRATION,
  GURO_PAR_LOGMODELS,
  GURO_PAR_LU2X2BLOCK,
  GURO_PAR_LUINTERVAL,
  GURO_PAR_MINBPFORBID,
  GURO_PAR_MIPBDSTRENGTH,
  GURO_PAR_MIPCLEANUP,
  GURO_PAR_MIPCOESTRENGTH,
  GURO_PAR_MOREPREINFO,
  GURO_PAR_NETIMPLIED,
  GURO_PAR_NODEPRE,
  GURO_PAR_NODEPROBE,
  GURO_PAR_NOIMPLIED,
  GURO_PAR_NOLOCALDISK,
  GURO_PAR_NONDEGENFLIP,
  GURO_PAR_OBJCUT,
  GURO_PAR_ONEROWLP,
  GURO_PAR_PAIRVARCUT,
  GURO_PAR_PARPRE,
  GURO_PAR_PREBOUND,
  GURO_PAR_PRECANCEL,
  GURO_PAR_PRECLIQSUB,
  GURO_PAR_PRECLIQUE,
  GURO_PAR_PRECOEFFICIENT,
  GURO_PAR_PRECOVERS,
  GURO_PAR_PREDOMINATE,
  GURO_PAR_PREDUALCONVTOBIN,
  GURO_PAR_PREDUALSLACK,
  GURO_PAR_PREGUBPROBE,
  GURO_PAR_PREKNAPLIFTLEN,
  GURO_PAR_PRELIMIT,
  GURO_PAR_PREMERGEVUBS,
  GURO_PAR_PREPAIRAGG,
  GURO_PAR_PREPARALLEL,
  GURO_PAR_PREPARTOBJ,
  GURO_PAR_PREPOWER,
  GURO_PAR_PREPREHEUR,
  GURO_PAR_PREPROBE,
  GURO_PAR_PREPROBE0,
  GURO_PAR_PREPWLRED,
  GURO_PAR_PRESINGLECOL,
  GURO_PAR_PRESPLITROWS,
  GURO_PAR_PRESUBROW,
  GURO_PAR_PRESUBSUME,
  GURO_PAR_PRETRANSRED,
  GURO_PAR_PRETWOROWS,
  GURO_PAR_PRIMALPARTIAL,
  GURO_PAR_PRIMDUALSWITCH,
  GURO_PAR_PROBECUTS,
  GURO_PAR_PWLRC,
  GURO_PAR_QCAGG,
  GURO_PAR_QCPCLEAN,
  GURO_PAR_QPREDUCE,
  GURO_PAR_QPSDADJUST,
  GURO_PAR_QPSDCHECK,
  GURO_PAR_QUICKDIVE,
  GURO_PAR_RCFIX,
  GURO_PAR_RECROSSOVER,
  GURO_PAR_REDUCEX,
  GURO_PAR_RELAXHEUR,
  GURO_PAR_RELAXPREP,
  GURO_PAR_ROOTBOUNDPROP,
  GURO_PAR_ROOTHELPER,
  GURO_PAR_ROOTPROBE,
  GURO_PAR_ROOTREPRE,
  GURO_PAR_SBFIX,
  GURO_PAR_SDISPINTERVAL,
  GURO_PAR_SIMPLEXALLOCS,
  GURO_PAR_SIMPLEXCRASH,
  GURO_PAR_SIMPLEXENDFAC,
  GURO_PAR_SIMPLEXPWRATIO,
  GURO_PAR_SOCPOUTER,
  GURO_PAR_SOLREDUCTION,
  GURO_PAR_SPXLHEIGHT,
  GURO_PAR_STMPFIXES,
  GURO_PAR_SYMMCUT,
  GURO_PAR_SYMMFIXPASSES,
  GURO_PAR_SYMMTRANS,
  GURO_PAR_UNITCOMMITMENT,
  GURO_PAR_UNSCALEDVIO,
  GURO_PAR_WARMBASIS,
  GURO_PAR_WEAKSYMMCUT,
  GURO_PAR_WORKERLOG,
  GURO_PAR_WRITEPREMODEL,
  IgnoreNames,
  IISMethod,
  ImpliedCuts,
  InfProofCuts,
  InfUnbdInfo,
  IntegralityFocus,
  JSONSolDetail,
  LazyConstraints,
  LogToConsole,
  Method,
  MinRelNodes,
  MIPFocus,
  MIPSepCuts,
  MIQCPMethod,
  MIRCuts,
  ModKCuts,
  MultiObjMethod,
  MultiObjPre,
  NetworkCuts,
  NodeMethod,
  NonConvex,
  NoRelHeuristic,
  NormAdjust,
  NumericFocus,
  ObjNumber,
  OutputFlag,
  PartitionPlace,
  PoolSearchMode,
  PoolSolutions,
  PreCrush,
  PreDepRow,
  PreDual,
  PreMIQCPForm,
  PrePasses,
  PreQLinearize,
  Presolve,
  PreSparsify,
  ProjImpliedCuts,
  PSDCuts,
  PumpPasses,
  QCPDual,
  Quad,
  Record,
  RelaxLiftCuts,
  RINS,
  RLTCuts,
  ScaleFlag,
  ScenarioNumber,
  Seed,
  ServerTimeout,
  Sifting,
  SiftMethod,
  SimplexPricing,
  SolutionLimit,
  SolutionNumber,
  StartNodeLimit,
  StartNumber,
  StrongCGCuts,
  SubMIPCuts,
  SubMIPNodes,
  Symmetry,
  Threads,
  TSPort,
  TuneCriterion,
  TuneJobs,
  TuneOutput,
  TuneResults,
  TuneTrials,
  UpdateMode,
  VarBranch,
  WorkerPort,
  ZeroHalfCuts,
  ZeroObjNodes
}

#[derive(Debug,Copy,Clone)]
pub enum DoubleParam {
  BarConvTol,
  BarQCPConvTol,
  BestBdStop,
  BestObjStop,
  CSQueueTimeout,
  Cutoff,
  FeasibilityTol,
  FeasRelaxBigM,
  FuncMaxVal,
  FuncPieceError,
  FuncPieceLength,
  FuncPieceRatio,
  GURO_PAR_ADAPTCONS,
  GURO_PAR_AGGMULTLIMIT,
  GURO_PAR_ASPIRE,
  GURO_PAR_BARSAFE,
  GURO_PAR_BIGM,
  GURO_PAR_BIGTOL,
  GURO_PAR_BINHEUR,
  GURO_PAR_BLINDHEUR,
  GURO_PAR_BUFFERRPC,
  GURO_PAR_CARDTWOOPT,
  GURO_PAR_CONCURRENTMODE,
  GURO_PAR_CPUAFFINITY,
  GURO_PAR_CUTDENSEFAC,
  GURO_PAR_CUTGROWTH,
  GURO_PAR_CUTNZGROWTH,
  GURO_PAR_CUTOFFPIVOTS,
  GURO_PAR_CUTORTHTOL,
  GURO_PAR_DISAGGMULTIMPL,
  GURO_PAR_DUALDEGENMOVES,
  GURO_PAR_FINGERPRINT,
  GURO_PAR_FORCEMIP,
  GURO_PAR_FORCESTART,
  GURO_PAR_HAILMARY,
  GURO_PAR_IGNOREHINTS,
  GURO_PAR_IISPRESOLVE,
  GURO_PAR_IISTOL,
  GURO_PAR_IMPROVEMETHOD,
  GURO_PAR_INFBOUND,
  GURO_PAR_INT2BINNODES,
  GURO_PAR_ITEST,
  GURO_PAR_KEEPLOCALE,
  GURO_PAR_LICENSEID,
  GURO_PAR_LURKBDHEUR,
  GURO_PAR_MAXTHREADS,
  GURO_PAR_MIPLIM,
  GURO_PAR_MIPLIMRATIO,
  GURO_PAR_MOREBQPCUTS,
  GURO_PAR_NORELHEURSOLS,
  GURO_PAR_NOSIGNALS,
  GURO_PAR_OBJNABSTOL,
  GURO_PAR_PAIRVARS,
  GURO_PAR_PARTRELAXANDFIXHEUR,
  GURO_PAR_PERMUTATIONSEED,
  GURO_PAR_PERMUTECOLS,
  GURO_PAR_PERMUTEROWS,
  GURO_PAR_PIPUSH,
  GURO_PAR_PIVOTTOL,
  GURO_PAR_PREBICONNECTED,
  GURO_PAR_PRECONNECTED,
  GURO_PAR_PREINFBND,
  GURO_PAR_PREINFOBJ,
  GURO_PAR_PRELIFTGROW,
  GURO_PAR_PREMAKEEQ,
  GURO_PAR_PREPWLIMIT,
  GURO_PAR_PREQSUBST,
  GURO_PAR_PRESS2MIP,
  GURO_PAR_PRESYMMPASSES,
  GURO_PAR_PREVUBLIFT,
  GURO_PAR_PROBEQUAD,
  GURO_PAR_PROCGROUPS,
  GURO_PAR_PWLMETHOD,
  GURO_PAR_PWLRCFCT,
  GURO_PAR_PWSLPLIMIT,
  GURO_PAR_RAMPUPNODES,
  GURO_PAR_RELSUBMIPCUTS,
  GURO_PAR_RHSSCALE,
  GURO_PAR_RINSDIVE,
  GURO_PAR_ROOTHEURMULT,
  GURO_PAR_ROOTQCCUTTHR,
  GURO_PAR_ROTCONEASBILIN,
  GURO_PAR_SINGLESCENARIO,
  GURO_PAR_SLAMHEUR,
  GURO_PAR_SOSBRRAT,
  GURO_PAR_SPXCONCURRENT,
  GURO_PAR_STRICTATTRFILE,
  GURO_PAR_TARGETMIPGAP,
  GURO_PAR_TIMESOFTLIMIT,
  GURO_PAR_TRICKLETOL,
  GURO_PAR_TUNEALLPARAMS,
  GURO_PAR_TUNEVARIANCEPEN,
  GURO_PAR_VTIMELIMIT,
  GURO_PAR_VTIMESOFTLIMIT,
  GURO_PAR_WLSTOKENDURATION,
  GURO_PAR_ZERO,
  GURO_PAR_ZEROOBJSTART,
  Heuristics,
  ImproveStartGap,
  ImproveStartNodes,
  ImproveStartTime,
  IntFeasTol,
  IterationLimit,
  MarkowitzTol,
  MIPGap,
  MIPGapAbs,
  NodefileStart,
  NodeLimit,
  NoRelHeurTime,
  NoRelHeurWork,
  ObjScale,
  OptimalityTol,
  PerturbValue,
  PoolGap,
  PoolGapAbs,
  PreSOS1BigM,
  PreSOS2BigM,
  PSDTol,
  TimeLimit,
  TuneCleanup,
  TuneTimeLimit
}

#[derive(Debug,Copy,Clone)]
pub enum StringParam {
  CloudAccessID,
  CloudHost,
  CloudPool,
  CloudSecretKey,
  ComputeServer,
  CSAPIAccessID,
  CSAPISecret,
  CSAppName,
  CSAuthToken,
  CSGroup,
  CSManager,
  CSRouter,
  Dummy,
  GURO_PAR_ISVAPPNAME,
  GURO_PAR_ISVKEY,
  GURO_PAR_ISVNAME,
  GURO_PAR_LICFILE,
  GURO_PAR_WLSACCESSID,
  GURO_PAR_WLSGENERATOR,
  GURO_PAR_WLSSECRET,
  GURO_PAR_WLSTOKEN,
  JobID,
  LogFile,
  NodefileDir,
  ResultFile,
  ServerPassword,
  SolFiles,
  TokenServer,
  Username,
  WorkerPassword,
  WorkerPool
}

#[derive(Debug,Copy,Clone)]
pub enum IntAttr {
  NumConstrs,
  NumVars,
  NumSOS,
  NumQConstrs,
  NumNZs,
  NumQNZs,
  NumQCNZs,
  NumIntVars,
  NumBinVars,
  NumPWLObjVars,
  ModelSense,
  IsMIP,
  IsQP,
  IsQCP,
  Status,
  SolCount,
  BarIterCount,
  VBasis,
  CBasis,
  PWLObjCvx,
  BranchPriority,
  VarPreStat,
  BoundVioIndex,
  BoundSVioIndex,
  ConstrVioIndex,
  ConstrSVioIndex,
  ConstrResidualIndex,
  ConstrSResidualIndex,
  DualVioIndex,
  DualSVioIndex,
  DualResidualIndex,
  DualSResidualIndex,
  ComplVioIndex,
  IntVioIndex,
  IISMinimal,
  IISLB,
  IISUB,
  IISConstr,
  IISSOS,
  IISQConstr,
  TuneResultCount,
  Lazy,
  VarHintPri
}

#[derive(Debug,Copy,Clone)]
pub enum CharAttr {
  VType,
  Sense,
  QCSense
}

#[derive(Debug,Copy,Clone)]
pub enum DoubleAttr {
  Runtime,
  ObjCon,
  LB,
  UB,
  Obj,
  Start,
  PreFixVal,
  RHS,
  QCRHS,
  MaxCoeff,
  MinCoeff,
  MaxBound,
  MinBound,
  MaxObjCoeff,
  MinObjCoeff,
  MaxRHS,
  MinRHS,
  ObjVal,
  ObjBound,
  ObjBoundC,
  MIPGap,
  IterCount,
  NodeCount,
  X,
  RC,
  Pi,
  QCPi,
  Slack,
  QCSlack,
  BoundVio,
  BoundSVio,
  BoundVioSum,
  BoundSVioSum,
  ConstrVio,
  ConstrSVio,
  ConstrVioSum,
  ConstrSVioSum,
  ConstrResidual,
  ConstrSResidual,
  ConstrResidualSum,
  ConstrSResidualSum,
  DualVio,
  DualSVio,
  DualVioSum,
  DualSVioSum,
  DualResidual,
  DualSResidual,
  DualResidualSum,
  DualSResidualSum,
  ComplVio,
  ComplVioSum,
  IntVio,
  IntVioSum,
  Kappa,
  KappaExact,
  SAObjLow,
  SAObjUp,
  SALBLow,
  SALBUp,
  SARHSLow,
  SAUBLow,
  SAUBUp,
  SARHSUp,
  Xn,
  FarkasProof,
  FarkasDual,
  UnbdRay,
  PStart,
  DStart,
  BarX,
  VarHintVal
}

#[derive(Debug,Copy,Clone)]
pub enum StringAttr {
  ModelName,
  VarName,
  ConstrName,
  QCName
}

macro_rules! impl_from {
  ($($t:ty)*) => ($(
    impl From<$t> for CString {
      fn from(attr: $t) -> CString {
        CString::new(format!("{:?}", attr).as_str()).unwrap()
      }
    }
  )*)
}

impl_from! { IntParam DoubleParam StringParam }
impl_from! { IntAttr CharAttr DoubleAttr StringAttr }


// Environment Creation and Destruction
extern "C" {
  pub fn GRBloadenv(envP: *mut *mut GRBenv, logfilename: c_str) -> c_int;

  pub fn GRBemptyenv(envP: *mut *mut GRBenv) -> c_int;

  pub fn GRBloadclientenv(envP: *mut *mut GRBenv, logfilename: c_str, computeserver: c_str, port: c_int,
                          password: c_str, priority: c_int, timeout: c_double)
                          -> c_int;

  pub fn GRBfreeenv(env: *mut GRBenv);

  pub fn GRBgetconcurrentenv(model: *mut GRBmodel, num: c_int) -> *mut GRBenv;

  pub fn GRBdiscardconcurrentenvs(model: *mut GRBmodel);
}

// Model Creation and Modification
extern "C" {
  pub fn GRBnewmodel(env: *mut GRBenv, modelP: *mut *mut GRBmodel, Pname: c_str, numvars: c_int,
                     obj: *const c_double, lb: *const c_double, ub: *const c_double, vtype: *const c_char,
                     varnames: *const c_str)
                     -> c_int;

  pub fn GRBcopymodel(model: *mut GRBmodel) -> *mut GRBmodel;

  pub fn GRBaddconstr(model: *mut GRBmodel, numnz: c_int, cind: *const c_int, cval: *const c_double, sense: c_char,
                      rhs: c_double, constrname: c_str)
                      -> c_int;

  pub fn GRBaddconstrs(model: *mut GRBmodel, numconstrs: c_int, numnz: c_int, cbeg: *const c_int, cind: *const c_int,
                       cval: *const c_double, sense: *const c_char, rhs: *const c_double, constrname: *const c_str)
                       -> c_int;

  pub fn GRBaddqconstr(model: *mut GRBmodel, numlnz: c_int, lind: *const c_int, lval: *const c_double, numqnz: c_int,
                       qrow: *const c_int, qcol: *const c_int, qval: *const c_double, sense: c_char, rhs: c_double,
                       QCname: c_str)
                       -> c_int;

  pub fn GRBaddqpterms(model: *mut GRBmodel, numqnz: c_int, qrow: *const c_int, qcol: *const c_int,
                       qval: *const c_double)
                       -> c_int;

  pub fn GRBaddrangeconstr(model: *mut GRBmodel, numnz: c_int, cind: *const c_int, cval: *const c_double,
                           lower: c_double, upper: c_double, constrname: c_str)
                           -> c_int;

  pub fn GRBaddrangeconstrs(model: *mut GRBmodel, numconstrs: c_int, numnz: c_int, cbeg: *const c_int,
                            cind: *const c_int, cval: *const c_double, lower: *const c_double,
                            upper: *const c_double, constrname: *const c_str)
                            -> c_int;

  pub fn GRBaddsos(model: *mut GRBmodel, numsos: c_int, nummembers: c_int, types: *const c_int, beg: *const c_int,
                   ind: *const c_int, weight: *const c_double)
                   -> c_int;

  pub fn GRBaddvar(model: *mut GRBmodel, numnz: c_int, vind: *const c_int, vval: *const c_double, obj: f64, lb: f64,
                   ub: f64, vtype: c_char, name: c_str)
                   -> c_int;

  pub fn GRBaddvars(model: *mut GRBmodel, numvars: c_int, numnz: c_int, vbeg: *const c_int, vind: *const c_int,
                    vval: *const c_double, obj: *const f64, lb: *const f64, ub: *const f64, vtype: *const c_char,
                    name: *const c_str)
                    -> c_int;

  pub fn GRBchgcoeffs(model: *mut GRBmodel, cnt: c_int, cind: *const c_int, vind: *const c_int, val: *const c_double)
                      -> c_int;

  pub fn GRBdelvars(model: *mut GRBmodel, numdel: c_int, ind: *const c_int) -> c_int;

  pub fn GRBdelconstrs(model: *mut GRBmodel, numdel: c_int, ind: *const c_int) -> c_int;

  pub fn GRBdelq(model: *mut GRBmodel) -> c_int;

  pub fn GRBdelqconstrs(model: *mut GRBmodel, len: c_int, ind: *const c_int) -> c_int;

  pub fn GRBdelsos(model: *mut GRBmodel, len: c_int, ind: *const c_int) -> c_int;

  pub fn GRBsetpwlobj(model: *mut GRBmodel, var: c_int, points: c_int, x: *const c_double, y: *const c_double)
                      -> c_int;

  pub fn GRBupdatemodel(model: *mut GRBmodel) -> c_int;

  pub fn GRBfreemodel(model: *mut GRBmodel) -> c_int;

// Xaddconstrs
// Xaddrangeconstrs
// Xaddvars
// Xchgcoeffs
// Xloadmodel
}

// Model Solution
extern "C" {

  pub fn GRBoptimize(model: *mut GRBmodel) -> c_int;

  pub fn GRBoptimizeasync(model: *mut GRBmodel) -> c_int;

  pub fn GRBcomputeIIS(model: *mut GRBmodel) -> c_int;

  pub fn GRBfeasrelax(model: *mut GRBmodel, relaxobjtype: c_int, minrelax: c_int, lbpen: *const c_double,
                      ubpen: *const c_double, rhspen: *const c_double, feasobjP: *const c_double)
                      -> c_int;

  pub fn GRBfixedmodel(model: *mut GRBmodel) -> *mut GRBmodel;

  pub fn GRBrelaxmodel(model: *mut GRBmodel) -> *mut GRBmodel;

  pub fn GRBpresolvemodel(model: *mut GRBmodel) -> *mut GRBmodel;

  pub fn GRBfeasibility(model: *mut GRBmodel) -> *mut GRBmodel;

  pub fn GRBresetmodel(model: *mut GRBmodel) -> c_int;

  pub fn GRBsync(model: *mut GRBmodel) -> c_int;
}

// Model Queries
extern "C" {
  pub fn GRBgetcoeff(model: *mut GRBmodel, constr: c_int, var: c_int, valP: *mut c_double) -> c_int;

  pub fn GRBgetconstrbyname(model: *mut GRBmodel, name: c_str, constrnumP: *mut c_int) -> c_int;

  pub fn GRBgetconstrs(model: *mut GRBmodel, numnzP: *mut c_int, cbeg: *mut c_int, cind: *mut c_int,
                       cval: *mut c_double, start: c_int, len: c_int)
                       -> c_int;

  pub fn GRBgetenv(model: *mut GRBmodel) -> *mut GRBenv;

  pub fn GRBgetpwlobj(model: *mut GRBmodel, var: c_int, npointsP: *mut c_int, x: *mut c_double, y: *mut c_double)
                      -> c_int;

  pub fn GRBgetq(model: *mut GRBmodel, numqnzP: *mut c_int, qrow: *mut c_int, qcol: *mut c_int, qval: *mut c_double)
                 -> c_int;

  pub fn GRBgetqconstr(model: *mut GRBmodel, qconstr: c_int, numlnzP: *mut c_int, lind: *mut c_int,
                       lval: *mut c_double, numqnzP: *mut c_int, qrow: *mut c_int, qcol: *mut c_int,
                       qval: *mut c_double)
                       -> c_int;

  pub fn GRBgetsos(model: *mut GRBmodel, nummembersP: *mut c_int, sostype: *mut c_int, beg: *mut c_int,
                   ind: *mut c_int, weight: *mut c_double, start: c_int, len: c_int)
                   -> c_int;

  pub fn GRBgetvarbyname(model: *mut GRBmodel, name: c_str, varnumP: *mut c_int) -> c_int;

  pub fn GRBgetvars(model: *mut GRBmodel, numnzP: *mut c_int, vbeg: *mut c_int, vind: *mut c_int,
                    vval: *mut c_double, start: c_int, len: c_int)
                    -> c_int;

// Xgetconstrs
// Xgetvars
}

// Input/Output
extern "C" {
  pub fn GRBreadmodel(env: *mut GRBenv, filename: c_str, modelP: *mut *mut GRBmodel) -> c_int;

  pub fn GRBread(model: *mut GRBmodel, filename: c_str) -> c_int;

  pub fn GRBwrite(model: *mut GRBmodel, filename: c_str) -> c_int;

}

extern "C" {
  pub fn GRBgetattrinfo(model: *mut GRBmodel, attrname: c_str, datatypeP: *mut c_int, attrtypeP: *mut c_int,
                        settableP: *mut c_int)
                        -> c_int;
}

extern "C" {
  pub fn GRBgetintattr(model: *mut GRBmodel, attrname: c_str, valueP: *mut c_int) -> c_int;

  pub fn GRBgetdblattr(model: *mut GRBmodel, attrname: c_str, valueP: *mut c_double) -> c_int;

  pub fn GRBgetstrattr(model: *mut GRBmodel, attrname: c_str, valueP: *mut c_str) -> c_int;


  pub fn GRBsetintattr(model: *mut GRBmodel, attrname: c_str, value: c_int) -> c_int;

  pub fn GRBsetdblattr(model: *mut GRBmodel, attrname: c_str, value: c_double) -> c_int;

  pub fn GRBsetstrattr(model: *mut GRBmodel, attrname: c_str, value: c_str) -> c_int;
}

extern "C" {
  pub fn GRBgetintattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, valueP: *mut c_int) -> c_int;

  pub fn GRBgetdblattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, valueP: *mut c_double) -> c_int;

  pub fn GRBgetcharattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, valueP: *mut c_char) -> c_int;

  pub fn GRBgetstrattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, valueP: *mut c_str) -> c_int;


  pub fn GRBsetintattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, value: c_int) -> c_int;

  pub fn GRBsetdblattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, value: c_double) -> c_int;

  pub fn GRBsetcharattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, value: c_char) -> c_int;

  pub fn GRBsetstrattrelement(model: *mut GRBmodel, attrname: c_str, element: c_int, value: c_str) -> c_int;
}

extern "C" {
  pub fn GRBgetintattrarray(model: *mut GRBmodel, attrname: c_str, first: c_int, len: c_int, values: *mut c_int)
                            -> c_int;

  pub fn GRBgetdblattrarray(model: *mut GRBmodel, attrname: c_str, first: c_int, len: c_int, values: *mut c_double)
                            -> c_int;

  pub fn GRBgetcharattrarray(model: *mut GRBmodel, attrname: c_str, first: c_int, len: c_int, values: *mut c_char)
                             -> c_int;

  pub fn GRBgetstrattrarray(model: *mut GRBmodel, attrname: c_str, first: c_int, len: c_int, values: *mut c_str)
                            -> c_int;


  pub fn GRBsetintattrarray(model: *mut GRBmodel, attrname: c_str, first: c_int, len: c_int, values: *const c_int)
                            -> c_int;

  pub fn GRBsetdblattrarray(model: *mut GRBmodel, attrname: c_str, first: c_int, len: c_int, values: *const c_double)
                            -> c_int;

  pub fn GRBsetcharattrarray(model: *mut GRBmodel, attrname: c_str, first: c_int, len: c_int, values: *const c_char)
                             -> c_int;

  pub fn GRBsetstrattrarray(model: *mut GRBmodel, attrname: *const c_char, first: c_int, len: c_int,
                            values: *const c_str)
                            -> c_int;
}

extern "C" {
  pub fn GRBgetintattrlist(model: *mut GRBmodel, attrname: c_str, len: c_int, ind: *const c_int, values: *mut c_int)
                           -> c_int;

  pub fn GRBgetdblattrlist(model: *mut GRBmodel, attrname: c_str, len: c_int, ind: *const c_int,
                           values: *mut c_double)
                           -> c_int;

  pub fn GRBgetcharattrlist(model: *mut GRBmodel, attrname: c_str, len: c_int, ind: *const c_int, values: *mut c_char)
                            -> c_int;

  pub fn GRBgetstrattrlist(model: *mut GRBmodel, attrname: c_str, len: c_int, ind: *const c_int, values: *mut c_str)
                           -> c_int;


  pub fn GRBsetintattrlist(model: *mut GRBmodel, attrname: c_str, len: c_int, ind: *const c_int, values: *const c_int)
                           -> c_int;

  pub fn GRBsetdblattrlist(model: *mut GRBmodel, attrname: c_str, len: c_int, ind: *const c_int,
                           values: *const c_double)
                           -> c_int;

  pub fn GRBsetcharattrlist(model: *mut GRBmodel, attrname: c_str, len: c_int, ind: *const c_int,
                            values: *const c_char)
                            -> c_int;

  pub fn GRBsetstrattrlist(model: *mut GRBmodel, attrname: *const c_char, len: c_int, ind: *const c_int,
                           values: *const c_str)
                           -> c_int;
}

// Parameter Management and Tuning
extern "C" {
  pub fn GRBtunemodel(model: *mut GRBmodel) -> c_int;

  pub fn GRBgettuneresult(model: *mut GRBmodel, n: c_int) -> c_int;

  pub fn GRBgetdblparam(env: *mut GRBenv, paramname: c_str, value: *mut c_double) -> c_int;

  pub fn GRBgetintparam(env: *mut GRBenv, paramname: c_str, value: *mut c_int) -> c_int;

  pub fn GRBgetstrparam(env: *mut GRBenv, paramname: c_str, value: *mut c_char) -> c_int;

  pub fn GRBsetdblparam(env: *mut GRBenv, paramname: c_str, value: c_double) -> c_int;

  pub fn GRBsetintparam(env: *mut GRBenv, paramname: c_str, value: c_int) -> c_int;

  pub fn GRBsetstrparam(env: *mut GRBenv, paramname: c_str, value: c_str) -> c_int;

  pub fn GRBgetdblparaminfo(env: *mut GRBenv, paramname: c_str, valueP: *mut c_double, minP: *mut c_double,
                            maxP: *mut c_double, defaultP: *mut c_double)
                            -> c_int;

  pub fn GRBgetintparaminfo(env: *mut GRBenv, paramname: c_str, valueP: *mut c_int, minP: *mut c_int,
                            maxP: *mut c_int, defaultP: *mut c_int)
                            -> c_int;

  pub fn GRBgetstrparaminfo(env: *mut GRBenv, paramname: c_str, valueP: *mut c_char, defaultP: *mut c_char) -> c_int;

  pub fn GRBreadparams(env: *mut GRBenv, filename: c_str) -> c_int;

  pub fn GRBwriteparams(env: *mut GRBenv, filename: c_str) -> c_int;
}

// Monitoring Progress - Logging and Callbacks
extern "C" {
  pub fn GRBmsg(env: *mut GRBenv, message: c_str);

  pub fn GRBsetcallbackfunc(model: *mut GRBmodel,
                            cb: extern "C" fn(*mut GRBmodel, *mut c_void, c_int, *mut c_void) -> c_int,
                            usrdata: *mut c_void)
                            -> c_int;

  pub fn GRBgetcallbackfunc(model: *mut GRBmodel,
                            cb: *mut extern "C" fn(*mut GRBmodel, *mut c_void, c_int, *mut c_void) -> c_int)
                            -> c_int;

  pub fn GRBcbget(cbdata: *mut c_void, where_: c_int, what: c_int, resultP: *mut c_void) -> c_int;

  pub fn GRBversion(majorP: *mut c_int, minorP: *mut c_int, technicalP: *mut c_int);
}

// Modifying Solver Behaviour - Callbacks
extern "C" {
  pub fn GRBcbcut(cbdata: *mut c_void, cutlen: c_int, cutind: *const c_int, cutval: *const c_double,
                  cutsense: c_char, cutrhs: c_double)
                  -> c_int;

  pub fn GRBcblazy(cbdata: *mut c_void, lazylen: c_int, lazyind: *const c_int, lazyval: *const c_double,
                   lazysense: c_char, lazyrhs: c_double)
                   -> c_int;

  pub fn GRBcbsolution(cbdata: *mut c_void, solution: *const c_double) -> c_int;

  pub fn GRBterminate(model: *mut GRBmodel);
}

// Error Handling
extern "C" {
  pub fn GRBgeterrormsg(env: *mut GRBenv) -> c_str;
}

// Advanced simplex routines
extern "C" {
  pub fn GRBFSolve(model: *mut GRBmodel, b: *mut GRBsvec, x: *mut GRBsvec) -> c_int;

  pub fn GRBBSolve(model: *mut GRBmodel, b: *mut GRBsvec, x: *mut GRBsvec) -> c_int;

  pub fn GRBBinvColj(model: *mut GRBmodel, j: c_int, x: *mut GRBsvec) -> c_int;

  pub fn GRBBinvRowi(model: *mut GRBmodel, i: c_int, x: *mut GRBsvec) -> c_int;

  pub fn GRBgetBasisHead(model: *mut GRBmodel, bhead: *mut c_int) -> c_int;
}

// vim: set foldmethod=syntax :
