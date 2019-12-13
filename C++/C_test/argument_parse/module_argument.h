#pragma once 

#include <list>
#include <string>

static const char DEFAULT_process_group_[] = "mainboard_default";
static const char DEFAULT_sched_name_[] = "CYBER_DEFAULT";

class ModuleArgument {
 public:
  ModuleArgument() = default;
  virtual ~ModuleArgument() = default;
  void DisplayUsage();
  void ParseArgument(int argc, char* const argv[]);
  void GetOptions(const int argc, char* const argv[]);
  const std::string& GetBinaryName() const;
  const std::string& GetProcessGroup() const;
  const std::string& GetSchedName() const;
  const std::list<std::string>& GetDAGConfList() const;

 private:
  std::list<std::string> dag_conf_list_;
  std::string binary_name_;
  std::string process_group_;
  std::string sched_name_;
};

inline const std::string& ModuleArgument::GetBinaryName() const {
  return binary_name_;
}

inline const std::string& ModuleArgument::GetProcessGroup() const {
  return process_group_;
}

inline const std::string& ModuleArgument::GetSchedName() const {
  return sched_name_;
}

inline const std::list<std::string>& ModuleArgument::GetDAGConfList() const {
  return dag_conf_list_;
}
