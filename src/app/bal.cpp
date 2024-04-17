/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021, Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <glog/logging.h>

#include "rootba/bal/ba_log_utils.hpp"
#include "rootba/bal/bal_app_options.hpp"
#include "rootba/ceres/bal_bundle_adjustment.hpp"
#include "rootba/cli/bal_cli_utils.hpp"
#include "rootba/solver/bal_bundle_adjustment.hpp"

int main(int argc, char** argv) {
  using namespace rootba;

  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  /*
   * 运行PoBA:
   * ./bin/bal --solver-type POWER_SCHUR_COMPLEMENT --input data/rootba/bal/ladybug/problem-49-7776-pre.txt
   * ./bin/bal_gui --solver-type POWER_SCHUR_COMPLEMENT --input data/rootba/bal/final/problem-93-61203-pre.txt
   * 
  To solve a single problem using either the PoBA solver or the PoBA preconditioner, you can specify the solver and preconditioner type.

```shell
# PoBA solver
./bin/bal --solver-type POWER_SCHUR_COMPLEMENT --input [PROBLEM]

# Explicit Schur complement solver with PoBA preconditioner
./bin/bal --solver-type SCHUR_COMPLEMENT --preconditioner-type POWER_SCHUR_COMPLEMENT --input [PROBLEM]
```
  */

  // 运行程序所用的命令行：
/*
  mkdir -p ../rootba_testing/qr32/
  mkdir -p ../rootba_testing/sc64/
  ./bin/bal -C ../rootba_testing/qr32/ --no-use-double --input ../../rootba/data/rootba/bal/ladybug/problem-49-7776-pre.txt
  ./bin/bal -C ../rootba_testing/sc64/ --solver-type SCHUR_COMPLEMENT --input ../../rootba/data/rootba/bal/ladybug/problem-49-7776-pre.txt
  ./scripts/plot-logs.py ../rootba_testing/
*/
  // ./bin/bal --dump-config --config /dev/null > rootba_config.toml // generate a config file with default values
  // ./bin/bal_gui --input data/rootba/bal/final/problem-93-61203-pre.txt
  // ./bin/bal --input data/rootba/bal/ladybug/problem-49-7776-pre.txt
  // ./bin/bal -C ../rootba_testing/qr32/ --no-use-double --input ../../rootba/data/rootba/bal/ladybug/problem-49-7776-pre.txt
  // -C表示输出日志到指定的的工作目录
  // --no-use-double表示使用单精度浮点数
  // --input表示输入数据集文件

  // parse cli and load config
  BalAppOptions options;
  if (!parse_bal_app_arguments(
          "Solve BAL problem with solver determined by config.", argc, argv,
          options)) {
    return 1;
  }

  /**
   * 打印options示例（基本都是缺省值）：
   * Options:
    [dataset]
    input = "data/rootba/bal/ladybug/problem-49-7776-pre.txt"
    input_type = "AUTO"
    save_output = false
    output_optimized_path = "optimized.cereal"
    normalize = true
    normalization_scale = 100.0
    rotation_sigma = 0.0
    translation_sigma = 0.0
    point_sigma = 0.0
    random_seed = 38401
    init_depth_threshold = 0.0
    quiet = false

    [solver]
    solver_type = "SQUARE_ROOT"
    verbosity_level = 2
    debug = false
    num_threads = 0
    optimized_cost = "ERROR"
    max_num_iterations = 20
    min_relative_decrease = 0.0
    initial_trust_region_radius = 10000.0
    min_trust_region_radius = 1e-32
    max_trust_region_radius = 1e+16
    min_lm_diagonal = 1e-06
    max_lm_diagonal = 1e+32
    min_linear_solver_iterations = 0
    max_linear_solver_iterations = 500
    eta = 0.1
    jacobi_scaling = true
    jacobi_scaling_epsilon = 0.0
    preconditioner_type = "SCHUR_JACOBI"
    linear_solver_type = "ITERATIVE_SCHUR"
    use_explicit_schur_complement = false
    function_tolerance = 1e-06
    gradient_tolerance = 0.0
    parameter_tolerance = 0.0
    check_gradients = false
    gradient_check_relative_precision = 1e-08
    gradient_check_numeric_derivative_relative_step_size = 1e-06
    use_double = true
    use_householder_marginalization = true
    staged_execution = true
    reduction_alg = 1
    power_order = 10
    initial_vee = 2.0
    vee_factor = 2.0

    [solver.residual]
    robust_norm = "NONE"
    huber_parameter = 1.0

    [solver.log]
    log_path = "ba_log.json"
    save_log_flags = [
    "JSON",
    ]
    disable_all = false # 参见 solver_options.hpp里面的 VISITABLE(BaLogOptions, log)
   */

  // print options
  if (options.solver.verbosity_level >= 2) {
    LOG(INFO) << "Options:\n" << options;
  }

  if (options.solver.solver_type == SolverOptions::SolverType::CERES) {
    DatasetSummary dataset_summary;
    PipelineTimingSummary timing_summary;
    BaLog log;

    // load data
    BalProblem<double> bal_problem = load_normalized_bal_problem<double>(
        options.dataset, &dataset_summary, &timing_summary);
    log_summary(log.static_data.timing, timing_summary);

    // run ceres solver (also updates `static_.timing`)
    bundle_adjust_ceres(bal_problem, options.solver, &log);

    // log summary (`static_.solver`, `static_.timing` and `iterations` are
    // already filled)
    log_summary(log.static_data.problem_info, dataset_summary);
    log.save_json(options.solver.log);
  } else {
    BalPipelineSummary summary;

    if (!options.solver.use_double) {
#ifdef ROOTBA_INSTANTIATIONS_FLOAT
      // load dataset
      auto bal_problem = load_normalized_bal_problem<float>(
          options.dataset, &summary.dataset, &summary.timing);

      // run solver
      bundle_adjust_manual(bal_problem, options.solver, &summary.solver,
                           &summary.timing);

      // postprocess
      bal_problem.postprocress(options.dataset, &summary.timing);
#else
      LOG(FATAL) << "Compiled without float support.";
#endif
    } else {
      // load_normalized_bal_problem调用的是bal_problem.hpp中的以下接口：
      /*
      BalProblem<Scalar> load_normalized_bal_problem(
        const BalDatasetOptions& options, DatasetSummary* dataset_summary = nullptr,
        PipelineTimingSummary* timing_summary = nullptr);
      */
      // bundle_adjust_manual在bal_bundle_adjustment.cpp中
#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
      // load dataset
      auto bal_problem = load_normalized_bal_problem<double>(
          options.dataset, &summary.dataset, &summary.timing);

      // run solver
      bundle_adjust_manual(bal_problem, options.solver, &summary.solver,
                           &summary.timing);

      // postprocess
      bal_problem.postprocress(options.dataset, &summary.timing);
#else
      LOG(FATAL) << "Compiled without double support.";
#endif
    }

    // log summary
    BaLog log;
    log_summary(log, summary);
    log.save_json(options.solver.log);
  }

  return 0;
}
