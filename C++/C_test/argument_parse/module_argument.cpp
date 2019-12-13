
#include "module_argument.h"
#include <iostream>
#include <getopt.h>
#include <libgen.h>

void ModuleArgument::DisplayUsage() {
  std::cout << "Usage: \n    " << binary_name_ << " [OPTION]...\n"
        << "Description: \n"
        << "    -h, --help : help information \n"
        << "    -d, --dag_conf=CONFIG_FILE : module dag config file\n"
        << "    -p, --process_group=process_group: the process "
           "namespace for running this module, default in manager process\n"
        << "    -s, --sched_name=sched_name: sched policy "
           "conf for hole process, sched_name should be conf in cyber.pb.conf\n"
        << "Example:\n"
        << "    " << binary_name_ << " -h\n"
        << "    " << binary_name_ << " -d dag_conf_file1 -d dag_conf_file2 "
        << "-p process_group -s sched_name\n";
}

void ModuleArgument::ParseArgument(const int argc, char* const argv[]) {
  binary_name_ = std::string(basename(argv[0]));
  GetOptions(argc, argv);

  if (process_group_.empty()) {
    process_group_ = DEFAULT_process_group_;
  }

  if (sched_name_.empty()) {
    sched_name_ = DEFAULT_sched_name_;
  }

  std::cout << "binary_name_ is " << binary_name_ << ", process_group_ is "
        << process_group_ << ", has " << dag_conf_list_.size() << " dag conf"<<std::endl;;
  for (std::string& dag : dag_conf_list_) {
    std::cout << "dag_conf: " << dag<<std::endl;
  }
}

void ModuleArgument::GetOptions(const int argc, char* const argv[]) {
  opterr = 0;  // extern int opterr
  int long_index = 0;
  const std::string short_opts = "hd:p:s:";
  static const struct option long_opts[] = {
      {"help", no_argument, nullptr, 'h'},
      {"dag_conf", required_argument, nullptr, 'd'},
      {"process_name", required_argument, nullptr, 'p'},
      {"sched_name", required_argument, nullptr, 's'},
      {NULL, no_argument, nullptr, 0}};

  // log command for info
  std::string cmd("");
  for (int i = 0; i < argc; ++i) {
    cmd += argv[i];
    cmd += " ";
  }
  std::cout << "command: " << cmd<<std::endl;

  do {
    int opt =
        getopt_long(argc, argv, short_opts.c_str(), long_opts, &long_index);
    if (opt == -1) {
      break;
    }

    std::cout<<opt<<"++++++++++++++++++"<<std::endl;
    switch (opt) {
      case 'd':
        dag_conf_list_.emplace_back(std::string(optarg));
        std::cout<<"ddddddddddd "<<optind<<" "<<optarg<<std::endl;
        for (int i = optind; i < argc; i++) {
          std::cout<<argc<<"================="<<argv[i]<<std::endl;
          if (*argv[i] != '-') { // for -d A.dag B.dag case
            dag_conf_list_.emplace_back(std::string(argv[i]));
          } else {
            break;
          }
        }
        break;
      case 'p':
        process_group_ = std::string(optarg);
        break;
      case 's':
        sched_name_ = std::string(optarg);
        break;
      case 'h':
        DisplayUsage();
        exit(0);
      default:
        break;
    }
  } while (true);
}